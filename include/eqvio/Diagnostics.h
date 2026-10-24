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

#pragma once

/** @brief What one vision update did, for the conditioning log.
 *
 * A feature count on its own says nothing about whether the visual update is
 * well posed: four features spread across the image and four features stacked
 * on one wall give the same count and very different geometry. The starvation
 * floor is stated as a feature count, so the claim that it keeps the update
 * well conditioned has to be checked against the update itself -- the rank and
 * condition number of the output matrix, the innovation covariance, and whether
 * a correction was applied at all.
 *
 * Filled in two places, because the numbers live in two places: VIO_eqf fills
 * the correction-step fields, VIOFilter fills the ones around it. A global,
 * following loopTimer, so the filter interface does not have to grow a
 * diagnostics argument that only the logger reads.
 *
 * Every field is reset at the start of each vision measurement, so `applied ==
 * false` after processVisionData means the update never reached the correction
 * step -- the filter ran open loop on the IMU for that frame.
 */
struct EqfDiagnostics {
    // --- set by VIOFilter::processVisionData ---
    bool integrated = false; ///< IMU integration to the image stamp succeeded.
    int nMeasIn = 0;         ///< Features handed to the filter by the front end.
    int nMeasUsed = 0;       ///< Features surviving the filter's own outlier rejection.
    int nOutliers = 0;       ///< Features the filter rejected (absolute or probabilistic).
    int nLandmarks = 0;      ///< Landmark states held after the update.
    double traceSigmaAtt = 0;    ///< trace of the attitude block of the Riccati matrix.
    double traceSigmaPos = 0;    ///< trace of the position block.
    double traceSigmaVel = 0;    ///< trace of the velocity block.
    double traceSigmaBias = 0;   ///< trace of the IMU bias blocks.
    double traceSigmaSensor = 0; ///< trace over the whole sensor state block.

    // --- set by VIO_eqf::performVisionUpdate ---
    bool applied = false;    ///< A correction was computed and applied.
    bool choleskyOk = false; ///< The innovation covariance factorised (it is PD).
    int measDim = 0;         ///< Rows of the output matrix, 2 per feature.
    int sensorRank = 0;      ///< Rank of the information this update adds to the 21-dim sensor state.
    double sensorCond = 0;   ///< Its condition number; inf if rank deficient.
    double sensorSigmaMin = 0; ///< Its smallest eigenvalue -- the least constrained direction.
    double sensorSigmaMax = 0; ///< Its largest.
    int motionRank = 0;        ///< Same, restricted to attitude, position and velocity: 0..9.
    double motionInfoMin = 0;  ///< Weakest constrained ego-motion direction.
    double innovationRms = 0;  ///< RMS of the measurement residual, in normalised image units.
    double innovationCovTrace = 0; ///< trace(S) / measDim, the mean predicted residual variance.
    double nis = 0; ///< yTilde' S^-1 yTilde: normalised innovation squared, chi-squared with measDim dof.
    bool rankRecovery = false; ///< The previous vision update was weak and outlier pruning was relaxed.

    void reset() { *this = EqfDiagnostics(); }
};

inline EqfDiagnostics eqfDiagnostics;
