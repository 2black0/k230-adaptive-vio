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

#include <numeric>

#include "opencv2/imgproc.hpp"

#include "eqvio/Diagnostics.h"
#include "eqvio/mathematical/EqFMatrices.h"
#include "eqvio/mathematical/VIO_eqf.h"

#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include "eigen3/Eigen/Eigenvalues"

void removeRows(Eigen::MatrixXd& mat, int startRow, int numRows) {
    int rows = mat.rows();
    int cols = mat.cols();

    assert(startRow + numRows <= rows);
    mat.block(startRow, 0, rows - numRows - startRow, cols) =
        mat.block(startRow + numRows, 0, rows - numRows - startRow, cols);
    mat.conservativeResize(rows - numRows, Eigen::NoChange);
}

void removeCols(Eigen::MatrixXd& mat, int startCol, int numCols) {
    int rows = mat.rows();
    int cols = mat.cols();

    assert(startCol + numCols <= cols);
    mat.block(0, startCol, rows, cols - numCols - startCol) =
        mat.block(0, startCol + numCols, rows, cols - numCols - startCol);
    mat.conservativeResize(Eigen::NoChange, cols - numCols);
}

void VIO_eqf::integrateObserverState(const IMUVelocity& imuVelocity, const double& dt, const bool& discreteLift) {

    // Integrate the state equation
    VIOGroup liftedVelocity;
    if (discreteLift) {
        liftedVelocity = liftVelocityDiscrete(stateEstimate(), imuVelocity, dt);
    } else {
        const auto liftedVelocityAlg = liftVelocity(stateEstimate(), imuVelocity);
        liftedVelocity = VIOExp(dt * liftedVelocityAlg);
    }
    assert(!liftedVelocity.hasNaN());
    X = X * liftedVelocity;
    assert(!X.hasNaN());
}

void VIO_eqf::integrateRiccatiStateFast(
    const IMUVelocity& imuVelocity, const double& dt,
    const Eigen::Matrix<double, IMUVelocity::CompDim, IMUVelocity::CompDim>& inputGainMatrix,
    const Eigen::MatrixXd& stateGainMatrix) {

    const Eigen::MatrixXd A0t = coordinateSuite->stateMatrixA(X, xi0, imuVelocity);
    Eigen::MatrixXd Bt = coordinateSuite->inputMatrixB(X, xi0);
    const Eigen::MatrixXd A0tExp = Eigen::MatrixXd::Identity(xi0.Dim(), xi0.Dim()) + dt * A0t;
    Sigma = A0tExp * Sigma * A0tExp.transpose() + dt * (Bt * inputGainMatrix * Bt.transpose() + stateGainMatrix);
    assert(!Sigma.hasNaN());
}

void VIO_eqf::integrateRiccatiStateAccurate(
    const IMUVelocity& imuVelocity, const double& dt,
    const Eigen::Matrix<double, IMUVelocity::CompDim, IMUVelocity::CompDim>& inputGainMatrix,
    const Eigen::MatrixXd& stateGainMatrix) {

    const Eigen::MatrixXd A0t = coordinateSuite->stateMatrixA(X, xi0, imuVelocity);
    Eigen::MatrixXd Bt = coordinateSuite->inputMatrixB(X, xi0);

    Eigen::MatrixXd AB = Eigen::MatrixXd::Zero(A0t.cols() + Bt.cols(), A0t.cols() + Bt.cols());
    AB.block(0, 0, A0t.rows(), A0t.cols()) = A0t;
    AB.block(0, A0t.rows(), Bt.rows(), Bt.cols()) = Bt;
    Eigen::MatrixXd ABExp = (dt * AB).exp();
    const Eigen::MatrixXd A0tExp = ABExp.block(0, 0, A0t.rows(), A0t.cols());
    const Eigen::MatrixXd BtExp = ABExp.block(0, A0t.rows(), Bt.rows(), Bt.cols());
    Sigma =
        A0tExp * Sigma * A0tExp.transpose() + BtExp * (inputGainMatrix / dt) * BtExp.transpose() + dt * stateGainMatrix;
    assert(!Sigma.hasNaN());
}

void VIO_eqf::integrateRiccatiStateDiscrete(
    const IMUVelocity& imuVelocity, const double& dt,
    const Eigen::Matrix<double, IMUVelocity::CompDim, IMUVelocity::CompDim>& inputGainMatrix,
    const Eigen::MatrixXd& stateGainMatrix) {

    Eigen::MatrixXd Bt = coordinateSuite->inputMatrixB(X, xi0);
    const Eigen::MatrixXd A0tDiscrete = coordinateSuite->stateMatrixADiscrete(X, xi0, imuVelocity, dt);
    Sigma =
        A0tDiscrete * Sigma * A0tDiscrete.transpose() + dt * (Bt * inputGainMatrix * Bt.transpose() + stateGainMatrix);
    assert(!Sigma.hasNaN());
}

void VIO_eqf::performVisionUpdate(
    const VisionMeasurement& measurement, const Eigen::MatrixXd& outputGainMatrix, const bool& useEquivariantOutput,
    const bool& discreteCorrection) {
    if (measurement.camCoordinates.empty())
        return;

    const VisionMeasurement estimatedMeasurement = measureSystemState(stateEstimate(), measurement.cameraPtr);
    const Eigen::VectorXd yTilde = measurement - estimatedMeasurement;
    const Eigen::MatrixXd Ct = coordinateSuite->outputMatrixC(xi0, X, measurement, useEquivariantOutput);

    // Fast and numerically stable Cholesky-based innovation update
    const Eigen::MatrixXd CtSigma = Ct * Sigma;
    const Eigen::MatrixXd S = CtSigma * Ct.transpose() + outputGainMatrix;
    const Eigen::LLT<Eigen::MatrixXd> S_llt(S);

    eqfDiagnostics.measDim = static_cast<int>(yTilde.size());
    eqfDiagnostics.innovationRms = yTilde.size() ? yTilde.norm() / std::sqrt((double)yTilde.size()) : 0.0;
    eqfDiagnostics.innovationCovTrace = S.rows() ? S.trace() / S.rows() : 0.0;

    Eigen::VectorXd Gamma;
    if (S_llt.info() == Eigen::Success) {
        const Eigen::VectorXd alpha = S_llt.solve(yTilde);
        // NIS: chi-squared with measDim degrees of freedom if the filter is
        // consistent. Free here -- alpha is already S^-1 yTilde.
        eqfDiagnostics.choleskyOk = true;
        eqfDiagnostics.nis = yTilde.dot(alpha);

        /* How much ego-motion this update actually constrains.
         *
         * The output map of this EqF touches the landmark states only: the
         * columns of Ct left of VIOSensorState::CompDim are structurally zero,
         * so asking for the rank of that block answers nothing. Pose, velocity
         * and bias are corrected indirectly, through their cross-covariance
         * with the landmarks, and the quantity that says whether they were
         * corrected is the information the update removes from their block:
         *
         *     G = [C Sigma]_sensor^T S^-1 [C Sigma]_sensor,
         *
         * which is exactly the reduction Sigma_sensor undergoes on this line.
         * G is positive semidefinite and 21x21 regardless of feature count; its
         * rank is the number of independent ego-motion directions the frame
         * constrained and its smallest eigenvalue is the weakest of them. This
         * is the measurement behind the starvation-floor claim -- a feature
         * count cannot distinguish four features that span the image from four
         * on the same wall, and G can.
         */
        const int sensorCols = std::min<int>(VIOSensorState::CompDim, CtSigma.cols());
        const Eigen::MatrixXd Ks = CtSigma.leftCols(sensorCols);
        const Eigen::MatrixXd G = Ks.transpose() * S_llt.solve(Ks);
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(G, Eigen::EigenvaluesOnly);
        const Eigen::VectorXd ev = es.eigenvalues(); // ascending
        const double eMax = ev.size() ? ev(ev.size() - 1) : 0.0;
        const double eMin = ev.size() ? std::max(ev(0), 0.0) : 0.0;
        // Eigen's own default rank tolerance, stated because it is part of what
        // the logged rank means.
        const double tol = eMax * G.rows() * Eigen::NumTraits<double>::epsilon();
        eqfDiagnostics.sensorRank = static_cast<int>((ev.array() > tol).count());
        eqfDiagnostics.sensorSigmaMax = eMax;
        eqfDiagnostics.sensorSigmaMin = eMin;
        eqfDiagnostics.sensorCond = eMin > 0 ? eMax / eMin : std::numeric_limits<double>::infinity();
        // The nine directions that are "ego-motion" in the ordinary sense:
        // attitude, position, velocity. Bias and camera extrinsics are slow
        // states and their conditioning is a different question.
        if (G.rows() >= 15) {
            const Eigen::MatrixXd Gm = G.block<9, 9>(6, 6);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> em(Gm, Eigen::EigenvaluesOnly);
            const Eigen::VectorXd mv = em.eigenvalues();
            const double mMax = mv(mv.size() - 1);
            eqfDiagnostics.motionRank =
                static_cast<int>((mv.array() > mMax * 9 * Eigen::NumTraits<double>::epsilon()).count());
            eqfDiagnostics.motionInfoMin = std::max(mv(0), 0.0);
        }

        Gamma = CtSigma.transpose() * alpha;

        const Eigen::MatrixXd S_inv_CtSigma = S_llt.solve(CtSigma);
        Sigma = Sigma - CtSigma.transpose() * S_inv_CtSigma;
        Sigma = 0.5 * (Sigma + Sigma.transpose());
    } else {
        const auto& SInv = S.inverse();
        const auto& K = Sigma * Ct.transpose() * SInv;
        eqfDiagnostics.nis = yTilde.transpose() * SInv * yTilde;
        Gamma = K * yTilde;
        Sigma = Sigma - K * Ct * Sigma;
        Sigma = 0.5 * (Sigma + Sigma.transpose());
    }
    assert(!Gamma.hasNaN());

    VIOGroup Delta;
    if (discreteCorrection) {
        Delta = coordinateSuite->liftInnovationDiscrete(Gamma, xi0);
    } else {
        Delta = VIOExp(coordinateSuite->liftInnovation(Gamma, xi0));
    }
    assert(!Delta.hasNaN());

    X = Delta * X;
    eqfDiagnostics.applied = true;

    assert(!Sigma.hasNaN());
    assert(!X.hasNaN());
}

VIOState VIO_eqf::stateEstimate() const { return stateGroupAction(X, xi0); }

VIOState VIO_eqf::predictState(const double& stamp, const std::vector<IMUVelocity> imuVelocities) const {
    VIOState statePrediction = stateEstimate();
    assert(stamp >= currentTime);
    for (size_t i = 0; i < imuVelocities.size(); ++i) {
        const double t0 = std::max(imuVelocities.at(i).stamp, this->currentTime);
        const double t1 = i + 1 < imuVelocities.size() ? std::min(imuVelocities.at(i + 1).stamp, stamp) : stamp;
        const double dt = std::max(t1 - t0, 0.0);

        statePrediction = integrateSystemFunction(statePrediction, imuVelocities.at(i), dt);
    }

    return statePrediction;
}

double VIO_eqf::computeNEES(const VIOState& trueState) const {
    VIOState truncatedState;
    truncatedState.sensor = trueState.sensor;
    for (const int& id : X.id) {
        const auto it =
            std::find_if(trueState.cameraLandmarks.begin(), trueState.cameraLandmarks.end(), [&id](const auto& lm) {
                return lm.id == id;
            });
        assert(it != trueState.cameraLandmarks.end());
        truncatedState.cameraLandmarks.emplace_back(*it);
    }

    const VIOState stateError = stateGroupAction(X.inverse(), truncatedState);
    const Eigen::VectorXd stateErrorLinearised = coordinateSuite->stateChart(stateError, xi0);
    const Eigen::MatrixXd informationMat = Sigma.inverse();
    const double NEES = stateErrorLinearised.transpose() * informationMat * stateErrorLinearised;
    return NEES / truncatedState.Dim();
}

void VIO_eqf::removeLandmarkByIndex(const int& idx) {
    xi0.cameraLandmarks.erase(xi0.cameraLandmarks.begin() + idx);
    X.id.erase(X.id.begin() + idx);
    X.Q.erase(X.Q.begin() + idx);
    removeRows(Sigma, VIOSensorState::CompDim + 3 * idx, 3);
    removeCols(Sigma, VIOSensorState::CompDim + 3 * idx, 3);
}

void VIO_eqf::removeMultipleLandmarksByIndices(const std::vector<int>& indices) {
    if (indices.empty()) return;
    if (indices.size() == 1) {
        removeLandmarkByIndex(indices[0]);
        return;
    }

    const int totalLandmarks = static_cast<int>(xi0.cameraLandmarks.size());
    std::vector<bool> toRemove(totalLandmarks, false);
    for (int idx : indices) {
        if (idx >= 0 && idx < totalLandmarks) {
            toRemove[idx] = true;
        }
    }

    std::vector<Landmark> newLandmarks;
    std::vector<int> newId;
    std::vector<liepp::SOT3d> newQ;
    const int newCount = totalLandmarks - static_cast<int>(indices.size());
    newLandmarks.reserve(newCount);
    newId.reserve(newCount);
    newQ.reserve(newCount);

    std::vector<int> keepIndices;
    keepIndices.reserve(VIOSensorState::CompDim + 3 * newCount);
    for (int i = 0; i < VIOSensorState::CompDim; ++i) {
        keepIndices.push_back(i);
    }

    for (int i = 0; i < totalLandmarks; ++i) {
        if (!toRemove[i]) {
            newLandmarks.push_back(std::move(xi0.cameraLandmarks[i]));
            newId.push_back(X.id[i]);
            newQ.push_back(std::move(X.Q[i]));
            keepIndices.push_back(VIOSensorState::CompDim + 3 * i);
            keepIndices.push_back(VIOSensorState::CompDim + 3 * i + 1);
            keepIndices.push_back(VIOSensorState::CompDim + 3 * i + 2);
        }
    }

    xi0.cameraLandmarks = std::move(newLandmarks);
    X.id = std::move(newId);
    X.Q = std::move(newQ);

    const int newDim = static_cast<int>(keepIndices.size());
    Eigen::MatrixXd newSigma(newDim, newDim);
    for (int r = 0; r < newDim; ++r) {
        const int orig_r = keepIndices[r];
        for (int c = 0; c < newDim; ++c) {
            newSigma(r, c) = Sigma(orig_r, keepIndices[c]);
        }
    }
    Sigma = std::move(newSigma);
}

void VIO_eqf::removeLandmarkById(const int& id) {
    const auto it = find_if(
        xi0.cameraLandmarks.begin(), xi0.cameraLandmarks.end(), [&id](const Landmark& lm) { return lm.id == id; });
    assert(it != xi0.cameraLandmarks.end());
    const int idx = distance(xi0.cameraLandmarks.begin(), it);
    removeLandmarkByIndex(idx);
}

Eigen::Matrix3d VIO_eqf::getLandmarkCovById(const int& id) const {
    const auto it = std::find_if(
        xi0.cameraLandmarks.begin(), xi0.cameraLandmarks.end(), [&id](const Landmark& lm) { return lm.id == id; });
    assert(it != xi0.cameraLandmarks.end());
    const int i = std::distance(xi0.cameraLandmarks.begin(), it);
    return Sigma.block<3, 3>(VIOSensorState::CompDim + 3 * i, VIOSensorState::CompDim + 3 * i);
}

Eigen::Matrix2d VIO_eqf::getOutputCovById(
    const int& id, [[maybe_unused]] const Eigen::Vector2d& y, const GIFT::GICameraPtr& camPtr) const {
    const Eigen::Matrix3d lmCov = getLandmarkCovById(id);
    const auto it = find_if(
        xi0.cameraLandmarks.begin(), xi0.cameraLandmarks.end(), [&id](const Landmark& lm) { return lm.id == id; });
    assert(it != xi0.cameraLandmarks.end());
    const auto it_X = find_if(X.id.begin(), X.id.end(), [&it](const int& i) { return i == it->id; });
    assert(it_X != X.id.end());
    const liepp::SOT3d& Q_i = X.Q[distance(X.id.begin(), it_X)];
    // const Matrix<double, 2, 3> C0i = settings->useEquivariantOutput
    //                                      ? coordinateSuite->outputMatrixCiStar(it->p, Q_i, camPtr, y)
    //                                      : coordinateSuite->outputMatrixCi(it->p, Q_i, camPtr);
    const Eigen::Matrix<double, 2, 3> C0i = coordinateSuite->outputMatrixCi(it->p, Q_i, camPtr);
    const Eigen::Matrix2d landmarkCov = C0i * lmCov * C0i.transpose();
    return landmarkCov;
}

void VIO_eqf::removeInvalidLandmarks() {
    std::set<int> invalidLandmarkIds;
    for (size_t i = 0; i < X.id.size(); ++i) {
        if (X.Q[i].a <= 1e-8 || X.Q[i].a > 1e8) {
            invalidLandmarkIds.emplace(X.id[i]);
        }
    }
    for (const int& lmId : invalidLandmarkIds) {
        removeLandmarkById(lmId);
    }
}

void VIO_eqf::addNewLandmarks(std::vector<Landmark>& newLandmarks, const Eigen::MatrixXd& newLandmarkCov) {
    xi0.cameraLandmarks.insert(xi0.cameraLandmarks.end(), newLandmarks.begin(), newLandmarks.end());

    std::vector<int> newIds(newLandmarks.size());
    std::transform(
        newLandmarks.begin(), newLandmarks.end(), newIds.begin(), [](const Landmark& blm) { return blm.id; });
    X.id.insert(X.id.end(), newIds.begin(), newIds.end());

    std::vector<liepp::SOT3d> newTransforms(newLandmarks.size());
    for (liepp::SOT3d& newTf : newTransforms) {
        newTf.setIdentity();
    }
    X.Q.insert(X.Q.end(), newTransforms.begin(), newTransforms.end());

    const int ogSize = Sigma.rows();
    const int newN = newLandmarks.size();
    Sigma.conservativeResize(ogSize + 3 * newN, ogSize + 3 * newN);
    Sigma.block(ogSize, 0, 3 * newN, ogSize).setZero();
    Sigma.block(0, ogSize, ogSize, 3 * newN).setZero();
    Sigma.block(ogSize, ogSize, 3 * newN, 3 * newN) = newLandmarkCov;
}

CSVLine& operator<<(CSVLine& line, const VIO_eqf& eqf) { return line << eqf.xi0 << eqf.X << eqf.Sigma; }