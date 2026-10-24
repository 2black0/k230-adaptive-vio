/*
    This file is part of EqVIO.

    EqVIO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    EqVIO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with EqVIO.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <algorithm>
#include <numeric>

#include "eigen3/Eigen/QR"

#include "eqvio/Diagnostics.h"
#include "eqvio/LoopTimer.h"
#include "eqvio/VIOFilter.h"
#include "eqvio/VIOFilterSettings.h"
#include "eqvio/mathematical/EqFMatrices.h"

using namespace Eigen;
using namespace std;
using namespace liepp;

VIOFilter::VIOFilter(const VIOFilter::Settings& settings) {
    this->settings = make_unique<VIOFilter::Settings>(settings);
    filterState.Sigma = this->settings->constructInitialStateCovariance();

    filterState.xi0.sensor.inputBias.setZero();
    filterState.xi0.sensor.pose.setIdentity();
    filterState.xi0.sensor.velocity.setZero();
    filterState.xi0.sensor.cameraOffset = settings.cameraOffset;

    filterState.coordinateSuite = getCoordinates(this->settings->coordinateChoice);
}

VIOFilter::VIOFilter(const VIOState& xi0, const VIOFilter::Settings& settings, const double& time) {
    this->settings = std::make_unique<VIOFilter::Settings>(settings);

    filterState.Sigma = this->settings->constructInitialStateCovariance(xi0.cameraLandmarks.size());
    filterState.xi0 = xi0;
    for (const Landmark& lm : xi0.cameraLandmarks) {
        filterState.X.Q.emplace_back(liepp::SOT3d::Identity());
        filterState.X.id.emplace_back(lm.id);
    }

    filterState.coordinateSuite = getCoordinates(settings.coordinateChoice);
    filterState.currentTime = time;
    initialisedFlag = true;
}

void VIOFilter::processIMUData(const IMUVelocity& imuVelocity) {
    if (!initialisedFlag) {
        /* Hold samples until the initialisation window is covered, then align
         * gravity against their mean. A negative window selects an IMU-only
         * adaptive window. The normal path remains short, but a large change
         * in acceleration direction during the first samples is evidence that
         * the first sample contains a transient specific force. In that case
         * the window is extended so the transient is averaged out. This keeps
         * fast-start behaviour for ordinary sequences while avoiding a large
         * roll/pitch error on a moving start. With a zero window the first
         * sample still reproduces the legacy behaviour. */
        initialisationBuffer.emplace_back(imuVelocity);
        const double span = initialisationBuffer.back().stamp - initialisationBuffer.front().stamp;
        if (settings->initialisationWindow < 0.0) {
            constexpr double minAdaptiveSpan = 0.02;
            constexpr double normalMaxAdaptiveSpan = 0.05;
            constexpr double transientMaxAdaptiveSpan = 0.50;
            constexpr double movingMagnitudeStd = 0.10;
            constexpr double transientTiltDeg = 3.0;
            if (span < minAdaptiveSpan)
                return;

            Vector3d meanAcc = Vector3d::Zero();
            for (const IMUVelocity& v : initialisationBuffer)
                meanAcc += v.acc;
            meanAcc /= static_cast<double>(initialisationBuffer.size());
            const Vector3d firstDir = initialisationBuffer.front().acc.normalized();
            const Vector3d meanDir = meanAcc.normalized();
            const double firstTiltDeg = std::acos(std::clamp(
                firstDir.dot(meanDir), -1.0, 1.0)) * 180.0 / M_PI;
            if (firstTiltDeg >= transientTiltDeg)
                adaptiveTransientDetected = true;
            const bool transientStart = adaptiveTransientDetected;
            const double maxAdaptiveSpan = transientStart
                ? transientMaxAdaptiveSpan : normalMaxAdaptiveSpan;
            const bool reachedMaximum = span >= maxAdaptiveSpan;
            double meanMagnitude = 0.0;
            for (const IMUVelocity& v : initialisationBuffer)
                meanMagnitude += v.acc.norm();
            meanMagnitude /= static_cast<double>(initialisationBuffer.size());
            double magnitudeVariance = 0.0;
            for (const IMUVelocity& v : initialisationBuffer) {
                const double d = v.acc.norm() - meanMagnitude;
                magnitudeVariance += d * d;
            }
            const double magnitudeStd = std::sqrt(magnitudeVariance /
                static_cast<double>(initialisationBuffer.size()));
            if (!reachedMaximum && (transientStart || magnitudeStd < movingMagnitudeStd))
                return;
        } else if (span < settings->initialisationWindow) {
            return;
        }
        /* Held samples are handed to the integrator afterwards, and the filter
         * clock is set to the FIRST of them, not the last. Otherwise widening
         * the window also delays the start, and start time is itself a
         * chaotically sensitive parameter on these sequences -- the measurement
         * would then confound the change in gravity direction with a change in
         * where the run begins, which is the thing it is supposed to isolate. */
        std::vector<IMUVelocity> held;
        held.swap(initialisationBuffer);
        initialisationBuffer = held;
        initialiseFromIMUBuffer();
        for (const IMUVelocity& v : held)
            velocityBuffer.emplace_back(v);
        return;
    }
    velocityBuffer.emplace_back(imuVelocity);
}

void VIOFilter::initialiseFromIMUData(const IMUVelocity& imuVelocity) {
    initialisationBuffer.clear();
    initialisationBuffer.emplace_back(imuVelocity);
    initialiseFromIMUBuffer();
}

/** @brief Fix the initial attitude from the buffered IMU window.
 *
 * The filter has no way to observe yaw at start-up, so initialisation only has
 * to supply roll and pitch, and those come from the direction of gravity:
 *
 *     acc ~ g R^T e_3   =>   e_3 ~ R acc.normalized()
 *
 * The approximation holds when the specific force is gravity alone. For a
 * single sample of handheld data it usually is not -- the measured direction is
 * tilted by whatever the hand was doing at that instant -- and the resulting
 * roll/pitch error appears as a constant horizontal component of gravity that
 * the filter integrates twice. A few degrees is metres of position within a
 * minute.
 *
 * Averaging over a window fixes this without requiring the device to be still.
 * Linear acceleration during ordinary walking changes sign on a sub-second
 * timescale, so its mean over a window of that order is small even where its
 * instantaneous value is large, while gravity is constant and survives the
 * average intact.
 */
void VIOFilter::initialiseFromIMUBuffer() {
    filterState.xi0.sensor.inputBias.setZero();
    filterState.xi0.sensor.pose.setIdentity();
    filterState.xi0.sensor.velocity.setZero();
    initialisedFlag = true;

    Vector3d meanAcc = Vector3d::Zero();
    for (const IMUVelocity& v : initialisationBuffer)
        meanAcc += v.acc;
    if (!initialisationBuffer.empty())
        meanAcc /= static_cast<double>(initialisationBuffer.size());

    const Vector3d approxGravity = meanAcc.normalized();
    filterState.xi0.sensor.pose.R = SO3d::SO3FromVectors(approxGravity, Vector3d::Unit(2));
    filterState.currentTime = initialisationBuffer.front().stamp;

    /* How far the window disagrees with the single sample it replaces. This is
     * the tilt error the original initialisation would have carried, in degrees;
     * it is the whole reason the window exists, so it is worth stating rather
     * than leaving to be inferred. */
    const Vector3d firstDir = initialisationBuffer.front().acc.normalized();
    const double tiltDeg = std::acos(std::clamp(firstDir.dot(approxGravity), -1.0, 1.0)) * 180.0 / M_PI;
    std::cout << "Initialised from " << initialisationBuffer.size() << " IMU sample(s) spanning "
              << (initialisationBuffer.back().stamp - initialisationBuffer.front().stamp)
              << " s; gravity direction differs from the first sample by " << tiltDeg << " deg."
              << std::endl;

    initialisationBuffer.clear();
}

void VIOFilter::setState(const VIOState& xi) {
    filterState.xi0 = xi;
    filterState.X = VIOGroup::Identity(xi.getIds());

    const int N = xi.cameraLandmarks.size();
    filterState.Sigma = MatrixXd::Identity(VIOSensorState::CompDim + 3 * N, VIOSensorState::CompDim + 3 * N);
    filterState.Sigma.block<VIOSensorState::CompDim, VIOSensorState::CompDim>(0, 0) =
        settings->constructInitialStateCovariance();
    filterState.Sigma.block(VIOSensorState::CompDim, VIOSensorState::CompDim, 3 * N, 3 * N) *=
        settings->initialPointVariance;

    initialisedFlag = true;
}

void VIOFilter::setLandmarks(const std::vector<Landmark>& cameraLandmarks) {

    Eigen::MatrixXd FullSigma0 = this->settings->constructInitialStateCovariance(cameraLandmarks.size());

    filterState.Sigma.block(
        VIOSensorState::CompDim, VIOSensorState::CompDim, cameraLandmarks.size() * 3, cameraLandmarks.size() * 3) =
        FullSigma0.block(
            VIOSensorState::CompDim, VIOSensorState::CompDim, cameraLandmarks.size() * 3, cameraLandmarks.size() * 3);

    filterState.xi0.cameraLandmarks = cameraLandmarks;
    filterState.X.Q.clear();
    filterState.X.id.clear();
    for (const Landmark& lm : cameraLandmarks) {
        filterState.X.Q.emplace_back(liepp::SOT3d::Identity());
        filterState.X.id.emplace_back(lm.id);
    }
}

void VIOFilter::augmentLandmarkStates(const std::vector<int>& newIds, const VIOState& providedState) {
    // Remove any landmarks not in newIds, and add new landmarks from the provided state
    removeOldLandmarks(newIds);

    // Identify which landmarks are new to the filter state
    std::vector<Landmark> newLandmarks;
    for (const int& id : newIds) {
        const auto it = std::find(filterState.X.id.begin(), filterState.X.id.end(), id);
        if (it != filterState.X.id.end()) {
            continue;
        }
        const auto it2 = std::find_if(
            providedState.cameraLandmarks.begin(), providedState.cameraLandmarks.end(),
            [&id](const auto& lm) { return lm.id == id; });
        newLandmarks.emplace_back(*it2);
    }

    const Eigen::MatrixXd newLandmarksCov =
        Eigen::MatrixXd::Identity(3 * newLandmarks.size(), 3 * newLandmarks.size()) * settings->initialPointVariance;
    filterState.addNewLandmarks(newLandmarks, newLandmarksCov);
}

bool VIOFilter::integrateUpToTime(const double& newTime) {
    if (newTime <= filterState.currentTime || filterState.currentTime < 0 || velocityBuffer.empty())
        return false;

    // The Riccati propagation happens first since it does not affect the state propagation.

    if (settings->fastRiccati) {
        double accumulatedTime = 0;

        IMUVelocity accumulatedVelocity = IMUVelocity::Zero();
        for (size_t i = 0; i < velocityBuffer.size(); ++i) {
            const double t0 = std::max(velocityBuffer.at(i).stamp, filterState.currentTime);
            const double t1 =
                i + 1 < velocityBuffer.size() ? std::min(velocityBuffer.at(i + 1).stamp, newTime) : newTime;
            const double dt = std::max(t1 - t0, 0.0);
            accumulatedTime += dt;
            accumulatedVelocity = accumulatedVelocity + velocityBuffer.at(i) * dt;
        }
        accumulatedVelocity = accumulatedVelocity * (1.0 / accumulatedTime);

        filterState.integrateRiccatiStateFast(
            accumulatedVelocity, accumulatedTime, settings->constructInputGainMatrix(),
            settings->constructStateGainMatrix(filterState.xi0.cameraLandmarks.size()));
    }

    // Now comes the state propagation
    for (size_t i = 0; i < velocityBuffer.size(); ++i) {
        const double t0 = std::max(velocityBuffer.at(i).stamp, filterState.currentTime);
        const double t1 = i + 1 < velocityBuffer.size() ? std::min(velocityBuffer.at(i + 1).stamp, newTime) : newTime;
        const double dt = std::max(t1 - t0, 0.0);

        if (!settings->fastRiccati && dt > 0) {
            if (settings->useDiscreteStateMatrix) {
                filterState.integrateRiccatiStateDiscrete(
                    velocityBuffer.at(i), dt, settings->constructInputGainMatrix(),
                    settings->constructStateGainMatrix(filterState.xi0.cameraLandmarks.size()));
            } else {
                filterState.integrateRiccatiStateAccurate(
                    velocityBuffer.at(i), dt, settings->constructInputGainMatrix(),
                    settings->constructStateGainMatrix(filterState.xi0.cameraLandmarks.size()));
            }
        }

        filterState.integrateObserverState(velocityBuffer.at(i), dt, settings->useDiscreteVelocityLift);
    }

    filterState.currentTime = newTime;

    // Clear velocity buffer from the front
    auto it = std::find_if(velocityBuffer.begin(), velocityBuffer.end(), [this](const IMUVelocity& imuVel) {
        return imuVel.stamp >= this->filterState.currentTime;
    });
    if (it != velocityBuffer.begin()) {
        --it;
        velocityBuffer.erase(velocityBuffer.begin(), it);
    }

    return true;
}

/** @brief Copy the Riccati diagonal blocks into the diagnostics record.
 *
 * The blocks are in the order constructInitialStateCovariance builds them: bias
 * gyro, bias accel, attitude, position, velocity, camera attitude, camera
 * position, then three columns per landmark. The trace of a block is the summed
 * variance of that part of the state -- a scalar that grows while the filter
 * runs on the IMU alone, which is what a starved visual update looks like from
 * the outside.
 */
void VIOFilter::recordCovarianceTraces() const {
    const Eigen::MatrixXd& Sig = filterState.Sigma;
    eqfDiagnostics.nLandmarks = static_cast<int>(filterState.X.id.size());
    if (Sig.rows() < VIOSensorState::CompDim)
        return;
    eqfDiagnostics.traceSigmaBias = Sig.block<6, 6>(0, 0).trace();
    eqfDiagnostics.traceSigmaAtt = Sig.block<3, 3>(6, 6).trace();
    eqfDiagnostics.traceSigmaPos = Sig.block<3, 3>(9, 9).trace();
    eqfDiagnostics.traceSigmaVel = Sig.block<3, 3>(12, 12).trace();
    eqfDiagnostics.traceSigmaSensor = Sig.block<VIOSensorState::CompDim, VIOSensorState::CompDim>(0, 0).trace();
}

void VIOFilter::processVisionData(const VisionMeasurement& measurement) {
    // The rank is computed inside the update, after this method's outlier
    // pruning. Carry the preceding rank across the reset so a rank-zero update
    // gets one subsequent frame in which the filter cannot discard most of the
    // visual evidence before the recovery update is assembled. The startup
    // rank-zero transient is excluded because all initial landmarks are new.
    // A short rank-0 blip is common in otherwise healthy sequences (the dev
    // split contains up to 9 consecutive frames, with fewer than 10 usable
    // features). Recovery is reserved for the sustained-information-loss
    // signature seen in F17: at least ten rank-0 updates while every update in
    // the streak still carried a substantial clean set.
    constexpr int rankRecoveryMinStreak = 10;
    constexpr int rankRecoveryMinUsableFeatures = 15;
    constexpr int rankRecoveryBurstFrames = 10;
    constexpr int rankRecoveryRearmStreak = 10;
    const bool rankRecoveryTrigger = settings->rankRecoveryRetention >= 0.0 &&
        processedVisionFrames > 10 && lastMotionRankValid &&
        zeroMotionRankStreak >= rankRecoveryMinStreak &&
        zeroMotionRankMinMeasUsed >= rankRecoveryMinUsableFeatures &&
        !rankRecoveryLatched;
    const bool rankRecovery = rankRecoveryTrigger || rankRecoveryFramesRemaining > 0;
    ++processedVisionFrames;
    // Reset first, so that every early return below leaves a record that says
    // the update did not happen rather than repeating the previous frame's.
    eqfDiagnostics.reset();
    eqfDiagnostics.rankRecovery = rankRecovery;
    eqfDiagnostics.nMeasIn = static_cast<int>(measurement.camCoordinates.size());

    // Use the stored velocity input to bring the filter up to the current timestamp
    loopTimer.startTiming("propagation");
    bool integrationFlag = integrateUpToTime(measurement.stamp);
    if (!integrationFlag || !initialisedFlag) {
        lastMotionRankValid = false;
        zeroMotionRankStreak = 0;
        zeroMotionRankMinMeasUsed = 0;
        return;
    }
    eqfDiagnostics.integrated = true;
    loopTimer.endTiming("propagation");

    loopTimer.startTiming("preprocessing");
    if (settings->removeLostLandmarks) {
        removeOldLandmarks(measurement.getIds());
        assert(measurement.camCoordinates.size() >= filterState.X.id.size());
        for (int i = filterState.X.id.size() - 1; i >= 0; --i) {
            assert(measurement.camCoordinates.count(filterState.X.id[i]) > 0);
        }
    }

    VisionMeasurement matchedMeasurement = measurement;
    const size_t nBeforeOutliers = matchedMeasurement.camCoordinates.size();
    removeOutliers(matchedMeasurement, rankRecovery);
    eqfDiagnostics.nOutliers = static_cast<int>(nBeforeOutliers - matchedMeasurement.camCoordinates.size());
    eqfDiagnostics.nMeasUsed = static_cast<int>(matchedMeasurement.camCoordinates.size());
    addNewLandmarks(matchedMeasurement);

    if (settings->removeLostLandmarks) {
        assert(matchedMeasurement.camCoordinates.size() == filterState.X.id.size());
        for (int i = filterState.X.id.size() - 1; i >= 0; --i) {
            assert(matchedMeasurement.camCoordinates.count(filterState.X.id[i]) > 0);
        }
    }
    loopTimer.endTiming("preprocessing");

    if (matchedMeasurement.camCoordinates.empty()) {
        // No correction this frame. Record the covariance anyway, so the log is
        // a continuous series rather than one with holes exactly where the
        // filter is running open loop.
        recordCovarianceTraces();
        lastMotionRankValid = false;
        zeroMotionRankStreak = 0;
        zeroMotionRankMinMeasUsed = 0;
        return;
    }

    // --------------------------
    // Compute the EqF innovation
    // --------------------------
    loopTimer.startTiming("correction");

    // Preserve the original fast path when no feature was soft-weighted.
    // Baseline, blanket, and selective rejection normally arrive here with an
    // empty noiseWeights map. Constructing a vector and a per-feature block
    // covariance for those modes needlessly made every non-adaptive run pay
    // for the adaptive backend extension. Only adaptive runs with at least
    // one explicit weight need the weighted covariance path.
    if (matchedMeasurement.noiseWeights.empty()) {
        filterState.performVisionUpdate(
            matchedMeasurement,
            settings->constructOutputGainMatrix(matchedMeasurement.camCoordinates.size()),
            settings->useEquivariantOutput, settings->useDiscreteInnovationLift);
    } else {
        // Per-feature measurement noise: the gate may soft-weight individual
        // features (inflating their noise instead of discarding them); features
        // without an explicit weight keep the configured sigma.
        std::vector<double> weights;
        weights.reserve(matchedMeasurement.camCoordinates.size());
        for (const auto& [id, coord] : matchedMeasurement.camCoordinates) {
            const auto it = matchedMeasurement.noiseWeights.find(id);
            weights.push_back(it != matchedMeasurement.noiseWeights.end() ? it->second : 1.0);
        }
        filterState.performVisionUpdate(
            matchedMeasurement, settings->constructOutputGainMatrix(weights),
            settings->useEquivariantOutput, settings->useDiscreteInnovationLift);
    }

    filterState.removeInvalidLandmarks();
    loopTimer.endTiming("correction");

    recordCovarianceTraces();
    lastMotionRank = eqfDiagnostics.motionRank;
    lastMotionRankValid = eqfDiagnostics.applied;
    if (rankRecovery) {
        // A weak recovery result must not cause a rescue loop: repeatedly
        // retaining every feature can make the innovation covariance worse.
        // Give the filter a contiguous burst. A single high-rank frame is not
        // enough to re-arm: F17 briefly reaches rank 9 and then collapses
        // again. Require a sustained normal high-rank run after the burst.
        if (rankRecoveryTrigger)
            rankRecoveryFramesRemaining = rankRecoveryBurstFrames - 1;
        else if (rankRecoveryFramesRemaining > 0)
            --rankRecoveryFramesRemaining;
        rankRecoveryLatched = true;
        rankRecoveryFullRankStreak = 0;
    } else if (rankRecoveryLatched && eqfDiagnostics.motionRank >= 8) {
        if (++rankRecoveryFullRankStreak >= rankRecoveryRearmStreak) {
            rankRecoveryLatched = false;
            rankRecoveryFullRankStreak = 0;
        }
    } else if (rankRecoveryLatched) {
        rankRecoveryFullRankStreak = 0;
    }
    if (eqfDiagnostics.applied && eqfDiagnostics.motionRank == 0) {
        ++zeroMotionRankStreak;
        zeroMotionRankMinMeasUsed = zeroMotionRankStreak == 1
            ? eqfDiagnostics.nMeasUsed
            : std::min(zeroMotionRankMinMeasUsed, eqfDiagnostics.nMeasUsed);
    } else {
        zeroMotionRankStreak = 0;
        zeroMotionRankMinMeasUsed = 0;
    }

    assert(!filterState.Sigma.hasNaN());
    assert(!filterState.X.hasNaN());
    // assert(filterState.Sigma.eigenvalues().real().minCoeff() > 0);
}

VIOState VIOFilter::stateEstimate() const { return filterState.stateEstimate(); }

const VIO_eqf& VIOFilter::viewEqFState() const { return filterState; }

VisionMeasurement VIOFilter::getFeaturePredictions(const GIFT::GICameraPtr& camPtr, const double& stamp) {
    if (settings->useFeaturePredictions) {
        return measureSystemState(filterState.predictState(stamp, velocityBuffer), camPtr);
    }
    return VisionMeasurement();
}

CSVLine& operator<<(CSVLine& line, const VIOFilter& filter) { return line << filter.filterState; }

double VIOFilter::getTime() const { return filterState.currentTime; }

void VIOFilter::addNewLandmarks(const VisionMeasurement& measurement) {
    // Grab all the new landmarks
    std::vector<Landmark> newLandmarks;
    for (const pair<const int, Vector2d>& cc : measurement.camCoordinates) {
        const int& ccId = cc.first;
        if (none_of(filterState.X.id.begin(), filterState.X.id.end(), [&ccId](const int& i) { return i == ccId; })) {
            Vector3d bearing = measurement.cameraPtr->undistortPoint(cc.second);
            newLandmarks.emplace_back(Landmark{bearing, ccId});
        }
    }
    if (newLandmarks.empty())
        return;

    // Initialise all landmarks to the median scene depth
    double initialDepth = settings->useMedianDepth ? getMedianSceneDepth() : settings->initialSceneDepth;
    for_each(newLandmarks.begin(), newLandmarks.end(), [&initialDepth](Landmark& blm) { blm.p *= initialDepth; });
    const int newN = newLandmarks.size();
    const Eigen::MatrixXd newLandmarksCov =
        Eigen::MatrixXd::Identity(3 * newN, 3 * newN) * settings->initialPointVariance;
    filterState.addNewLandmarks(newLandmarks, newLandmarksCov);
}

void VIOFilter::removeOldLandmarks(const vector<int>& measurementIds) {
    // Determine which indices have been lost
    vector<int> lostIndices(filterState.X.id.size());
    iota(lostIndices.begin(), lostIndices.end(), 0);
    if (lostIndices.empty())
        return;

    const auto lostIndicesEnd = remove_if(lostIndices.begin(), lostIndices.end(), [&](const int& lidx) {
        const int& oldId = filterState.X.id[lidx];
        return any_of(
            measurementIds.begin(), measurementIds.end(), [&oldId](const int& measId) { return measId == oldId; });
    });
    lostIndices.erase(lostIndicesEnd, lostIndices.end());

    if (lostIndices.empty())
        return;

    // Remove the origin state and transforms and filterState.Sigma bits corresponding to these indices in a single batch.
    reverse(lostIndices.begin(), lostIndices.end()); // Should be in descending order now
    filterState.removeMultipleLandmarksByIndices(lostIndices);
}

void VIOFilter::removeOutliers(VisionMeasurement& measurement, bool rankRecovery) {
    const double retention = rankRecovery
        ? std::max(settings->featureRetention, settings->rankRecoveryRetention)
        : settings->featureRetention;
    const size_t maxOutliers = (1.0 - std::clamp(retention, 0.0, 1.0)) * measurement.camCoordinates.size();
    const VIOState xiHat = stateEstimate();
    const VisionMeasurement yHat = measureSystemState(xiHat, measurement.cameraPtr);
    // Remove if the difference between the true and expected measurement exceeds a threshold
    std::vector<int> proposedOutliers;
    std::map<int, double> absoluteOutliers;
    for (const auto& [lmId, yHat_i] : yHat.camCoordinates) {
        if (measurement.camCoordinates.count(lmId) == 0) {
            continue;
        }
        double bearingErrorAbs = (measurement.camCoordinates.at(lmId) - yHat_i).norm();
        if (bearingErrorAbs > settings->outlierThresholdAbs) {
            absoluteOutliers[lmId] = bearingErrorAbs;
            proposedOutliers.emplace_back(lmId);
        }
    }

    std::map<int, double> probabilisticOutliers;
    const VisionMeasurement& measurementResidual = measurement - yHat;
    for (const auto& [lmId, yTilde_i] : measurementResidual.camCoordinates) {
        if (absoluteOutliers.count(lmId) || measurement.camCoordinates.count(lmId) == 0) {
            continue;
        }
        const Matrix2d outputCov =
            filterState.getOutputCovById(lmId, measurement.camCoordinates[lmId], measurement.cameraPtr);
        double bearingErrorProb = yTilde_i.transpose() * outputCov.inverse() * yTilde_i;
        if (bearingErrorProb > settings->outlierThresholdProb) {
            probabilisticOutliers[lmId] = bearingErrorProb;
            proposedOutliers.emplace_back(lmId);
        }
    }

    // Prioritise which landmarks to discard
    std::sort(
        proposedOutliers.begin(), proposedOutliers.end(),
        [&absoluteOutliers, &probabilisticOutliers](const int& lmId1, const int& lmId2) {
            if (absoluteOutliers.count(lmId1)) {
                if (absoluteOutliers.count(lmId2)) {
                    return absoluteOutliers.at(lmId1) < absoluteOutliers.at(lmId2);
                } else {
                    return false;
                }
            } else {
                if (absoluteOutliers.count(lmId2)) {
                    return true;
                } else {
                    return probabilisticOutliers.at(lmId1) < probabilisticOutliers.at(lmId2);
                }
            }
        });
    std::reverse(proposedOutliers.begin(), proposedOutliers.end());
    if (proposedOutliers.size() > maxOutliers) {
        proposedOutliers.erase(proposedOutliers.begin() + maxOutliers, proposedOutliers.end());
    }

    for_each(proposedOutliers.begin(), proposedOutliers.end(), [this, &measurement](const int& lmId) {
        filterState.removeLandmarkById(lmId);
        measurement.camCoordinates.erase(lmId);
    });
}

double VIOFilter::getMedianSceneDepth() const {
    const vector<Landmark> landmarks = this->stateEstimate().cameraLandmarks;
    vector<double> depthsSquared(landmarks.size());
    transform(landmarks.begin(), landmarks.end(), depthsSquared.begin(), [](const Landmark& blm) {
        return blm.p.squaredNorm();
    });
    const auto midway = depthsSquared.begin() + depthsSquared.size() / 2;
    nth_element(depthsSquared.begin(), midway, depthsSquared.end());
    double medianDepth = settings->initialSceneDepth;
    if (!(midway == depthsSquared.end())) {
        medianDepth = pow(*midway, 0.5);
    }

    return medianDepth;
}
