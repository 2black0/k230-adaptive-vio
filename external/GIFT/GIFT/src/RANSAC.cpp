/*
    This file is part of GIFT.

    GIFT is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    GIFT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with GIFT.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "GIFT/RANSAC.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include <random>

namespace {

// Row of the 8-point design matrix. Kept as the exact float/double expression
// of the original fitEssentialMatrix so results are bit-identical.
void fillDesignRow(Eigen::MatrixXT& A, size_t row, const cv::Point2f& p2, const GIFT::Feature& f) {
    const cv::Point2f p1 = p2 - cv::Point2f(f.opticalFlowNorm.x(), f.opticalFlowNorm.y());
    A.row(row) << p1.x * p2.x, p1.x * p2.y, p1.x, p1.y * p2.x, p1.y * p2.y, p1.y, p2.x, p2.y, 1;
}

Eigen::Matrix3T essentialFromDesign(const Eigen::MatrixXT& A) {
    // The initial solution is found using an SVD of A.
    const Eigen::JacobiSVD<Eigen::Matrix<ftype, Eigen::Dynamic, 9>> svdA(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Matrix<ftype, 9, 1> solutionVec = svdA.matrixV().template rightCols<1>();
    const Eigen::Matrix3T solutionMat = Eigen::Matrix3T(solutionVec.data()).transpose();

    // The final solution is found by projecting to the nearest essential matrix.
    const Eigen::JacobiSVD<Eigen::Matrix3T> svdB(solutionMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const Eigen::Vector3T& singularVals = svdB.singularValues();
    const ftype avgSingularVal = 0.5 * (singularVals(0) + singularVals(1));
    const Eigen::Matrix3d essentialMat = svdB.matrixU() *
                                         Eigen::DiagonalMatrix<ftype, 3>(avgSingularVal, avgSingularVal, 0.0) *
                                         svdB.matrixV().transpose();
    return essentialMat;
}

} // namespace

std::vector<GIFT::Feature> GIFT::determineStaticWorldInliers(
    const std::vector<GIFT::Feature>& features, const GIFT::RansacParameters& params, std::mt19937& generator) {
    std::vector<GIFT::Feature> bestFitInliers;

    // fitEssentialMatrix below is the 8-point algorithm, so a sample smaller
    // than 8 leaves A (n x 9) with a nullspace of dimension 9-n >= 2 and the
    // "solution" is an arbitrary vector from it. The K230 configs ship
    // minDataPoints: 5, which is silently degenerate, so clamp it here rather
    // than trust the yaml.
    const size_t sampleSize = std::max<size_t>(params.minDataPoints, 8);

    // Requiring at least minInliers features *before* starting means the whole
    // stage is a no-op whenever the tracker carries fewer than that. With
    // minInliers: 30 and 15-25 tracked features it never ran at all. The real
    // requirement is only that a sample can be drawn.
    if (features.size() < sampleSize || params.maxIterations == 0) {
        return features;
    }
    const size_t minInliers = std::min(params.minInliers, features.size());

    // camCoordinatesNorm() undistorts on every call, which is iterative for
    // fisheye models; undistort each feature once instead of once per
    // iteration. Indices are sampled with the same reservoir sampler, so the
    // RNG draws and sample order are unchanged.
    std::vector<cv::Point2f> normCoords(features.size());
    std::vector<size_t> indices(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
        normCoords[i] = features[i].camCoordinatesNorm();
        indices[i] = i;
    }

    Eigen::MatrixXT A(sampleSize, 9);
    for (size_t iter = 0; iter < params.maxIterations; ++iter) {
        const std::vector<size_t> sampled = sampleVector(indices, sampleSize, generator);
        assert(sampled.size() == sampleSize);
        for (size_t r = 0; r < sampled.size(); ++r) {
            fillDesignRow(A, r, normCoords[sampled[r]], features[sampled[r]]);
        }
        const Eigen::Matrix3T sampleEssentialMatrix = essentialFromDesign(A);

        // Determine inliers
        std::vector<GIFT::Feature> modelInliers;
        modelInliers.reserve(features.size());
        for (size_t i = 0; i < features.size(); ++i) {
            const GIFT::Feature& f = features[i];
            const cv::Point2f& p2CV = normCoords[i];
            const Eigen::Vector3T& p2 = Eigen::Vector3T(p2CV.x, p2CV.y, 1.0);
            const Eigen::Vector3T& p1 = p2 - Eigen::Vector3T(f.opticalFlowNorm.x(), f.opticalFlowNorm.y(), 0.0);

            // The epipolar residual is signed; testing it unsigned accepted
            // every outlier that happened to fall on the negative side,
            // which is about half of them.
            const ftype error = p1.transpose() * sampleEssentialMatrix * p2;
            if (std::abs(error) < params.inlierThreshold) {
                modelInliers.push_back(f);
            }
        }

        // The best model is determined by the highest number of inliers
        if (modelInliers.size() > bestFitInliers.size() && modelInliers.size() >= minInliers) {
            bestFitInliers = modelInliers;
        }
    }

    if (bestFitInliers.size() < minInliers) {
        return features;
    }
    return bestFitInliers;
}

Eigen::Matrix3T GIFT::fitEssentialMatrix(const std::vector<GIFT::Feature>& features) {
    // This uses the 8-point algorithm.
    // The resulting essential matrix E is expected to satisfy
    // p_{t-1}^\top E p_t = 0, for every feature point p.

    // Populate the matrix A that satisfies A f = 0, where f are the vectorised matrix entries of the solution.
    Eigen::MatrixXT A(features.size(), 9);
    for (size_t i = 0; i < features.size(); ++i) {
        fillDesignRow(A, i, features[i].camCoordinatesNorm(), features[i]);
    }
    return essentialFromDesign(A);
}
