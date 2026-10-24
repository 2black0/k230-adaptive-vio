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

#include "GIFT/PointFeatureTracker.h"
#include "eigen3/Eigen/SVD"
#include "eqvio/LoopTimer.h"
#include "iostream"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include "string"

using namespace GIFT;
using namespace Eigen;
using namespace std;
using namespace cv;

namespace {
// The tracker is also built into applications that register no timing labels at
// all, so every stage timer is conditional on the label existing. Testing a
// std::map for a key costs far less than the stage being timed.
struct StageTimer {
    const std::string label;
    const bool on;
    explicit StageTimer(const char* l) : label(l), on(loopTimer.has(l)) {
        if (on)
            loopTimer.startTiming(label);
    }
    ~StageTimer() {
        if (on)
            loopTimer.endTiming(label);
    }
};
} // namespace

void PointFeatureTracker::processImage(const Mat& image, const std::map<int, cv::Point2f>& predictedFeatures) {
    // Equalise the image histogram if desired
    cv::Mat equalisedImage = image;
    if (settings.claheClipLimit > 0) {
        cv::Mat greyImage = image;
        if (image.channels() > 1) {
            cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);
        }
        const double meanIntensity = cv::mean(greyImage)[0];
        settings.claheMeanState =
            settings.claheMeanState < 0 ? meanIntensity : 0.05 * meanIntensity + 0.95 * settings.claheMeanState;
        if (settings.claheMinMeanIntensity <= 0 || settings.claheMeanState >= settings.claheMinMeanIntensity) {
            auto clahe = cv::createCLAHE(settings.claheClipLimit, cv::Size(8, 8));
            clahe->apply(greyImage, equalisedImage);
        }
    } else if (settings.equaliseImageHistogram) {
        cv::Mat greyImage = image;
        if (image.channels() > 1) {
            cv::cvtColor(image, greyImage, cv::COLOR_BGR2GRAY);
        }
        cv::equalizeHist(greyImage, equalisedImage);
    }

    {
        StageTimer t("klt");
        this->trackFeatures(equalisedImage, predictedFeatures);
    }
    this->previousImage = equalisedImage;

    if (settings.ransacParams.maxIterations > 0) {
        StageTimer t("static ransac");
        const auto inlierFeatures = determineStaticWorldInliers(features, settings.ransacParams, settings.rng);
        if (inlierFeatures.size() > 0) {
            features = inlierFeatures;
        }
    }

    if (this->features.size() > this->settings.featureSearchThreshold * this->settings.maxFeatures)
        return;

    StageTimer t("detect");
    detectFeatures(equalisedImage);
}

void PointFeatureTracker::detectFeatures(const Mat& image) {
    vector<Point2f> newPoints = this->identifyFeatureCandidates(image);

    vector<Feature> newFeatures(newPoints.size());
    std::transform(newPoints.begin(), newPoints.end(), newFeatures.begin(),
        [this](const cv::Point2f& pt) { return Feature(pt, this->cameraPtr, -1); });

    this->addNewFeatures(newFeatures);
}

void PointFeatureTracker::trackFeatures(const Mat& image, const std::map<int, cv::Point2f>& predictedFeatures) {
    if (features.empty())
        return;

    vector<Point2f> oldPoints;
    for (const auto& feature : features) {
        oldPoints.emplace_back(feature.camCoordinates);
    }

    vector<Point2f> points;
    for (const auto& feature : features) {
        const auto& predIt = predictedFeatures.find(feature.idNumber);
        if (predIt != predictedFeatures.end()) {
            points.emplace_back(predIt->second);
        } else {
            points.emplace_back(feature.camCoordinates);
        }
    }

    vector<uchar> status;
    vector<float> err;
    if (predictedFeatures.empty()) {
        points.clear();
        calcOpticalFlowPyrLK(previousImage, image, oldPoints, points, status, err,
            Size(settings.winSize, settings.winSize), settings.maxLevel,
            TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, (0.01000000000000000021)));
    } else {
        calcOpticalFlowPyrLK(previousImage, image, oldPoints, points, status, err,
            Size(settings.winSize, settings.winSize), settings.maxLevel,
            TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, (0.01000000000000000021)),
            cv::OPTFLOW_USE_INITIAL_FLOW);
    }

    // Remove features lost in tracking
    for (long int i = points.size() - 1; i >= 0; --i) {
        if (status[i] == 0 || err[i] >= settings.maxError) {
            features.erase(features.begin() + i);
            continue;
        }

        // A tracked point can leave the image: calcOpticalFlowPyrLK reports
        // status 1 for extrapolated locations, and here they reach x = -5.9 and
        // x = 380.9 on a 376-wide frame, about 1 % of observations. Such a
        // feature has no measurement behind it, and indexing the mask with it
        // reads outside the buffer -- which silently perturbed every masked run
        // relative to an unmasked one, even when the mask was entirely zero.
        const cv::Point2f& pt = points[i];
        if (pt.x < 0 || pt.y < 0 || pt.x >= image.cols || pt.y >= image.rows) {
            features.erase(features.begin() + i);
            continue;
        }

        if (!mask.empty()) {
            if (mask.at<uchar>(cv::Point(cvRound(pt.x), cvRound(pt.y))) == 0) {
                features.erase(features.begin() + i);
                continue;
            }
        }

        features[i].update(points[i]);
    }

    // rejectStaticWorldOutliers(features);

    removeFeaturesTooClose(features, settings.trackedFeatureDist);
}

vector<Point2f> PointFeatureTracker::identifyFeatureCandidates(const Mat& image) const {
    Mat imageGrey;
    if (image.channels() > 1)
        [[unlikely]] { cv::cvtColor(image, imageGrey, cv::COLOR_BGR2GRAY); }
    else {
        imageGrey = image;
    }

    vector<Point2f> proposedFeatures;
    if (settings.useFastFeatures){
        std::vector<cv::KeyPoint> fastFeatures;
        cv::FAST(imageGrey, fastFeatures, 10, true);
        proposedFeatures.resize(fastFeatures.size());
        std::transform(fastFeatures.begin(), fastFeatures.end(), proposedFeatures.begin(),
            [](const cv::KeyPoint& kp){return kp.pt;}
        );
    } else {
    goodFeaturesToTrack(
        imageGrey, proposedFeatures, settings.maxFeatures, settings.minHarrisQuality, settings.featureDist, mask);
    }


    vector<Point2f> newFeatures = this->removeDuplicateFeatures(proposedFeatures);

    return newFeatures;
}

vector<Point2f> PointFeatureTracker::removeDuplicateFeatures(const vector<Point2f>& proposedFeatures) const {
    vector<Point2f> newFeatures;
    for (const auto& proposedFeature : proposedFeatures) {
        bool useFlag = true;
        for (const auto& feature : this->features) {
            if (norm(proposedFeature - feature.camCoordinates) < settings.featureDist) {
                useFlag = false;
                break;
            }
        }

        if (useFlag) {
            newFeatures.emplace_back(proposedFeature);
        }
    }
    return newFeatures;
}

void PointFeatureTracker::addNewFeatures(vector<Feature> newFeatures) {
    for (auto& lm : newFeatures) {
        if (features.size() >= settings.maxFeatures)
            break;

        lm.idNumber = ++currentNumber;
        features.emplace_back(lm);
    }
}

void PointFeatureTracker::removeFeaturesTooClose(std::vector<Feature>& features, const ftype& closeDist) {
    // Removes features that are closer than closeDist.
    // Keep the feature with the longest lifetime.

    if (closeDist <= 0.0) {
        return;
    }

    ftype closeDist2 = closeDist * closeDist;
    for (int i = features.size() - 1; i >= 0; --i) {
        Feature& fi = features[i];
        for (int j = i - 1; j >= 0; --j) {
            Feature& fj = features[j];
            if (fi.lifetime > fj.lifetime) {
                // If fi is older than fj, fi will not be removed now.
                continue;
            }
            ftype dist2 = (fi.camCoordinates - fj.camCoordinates).dot(fi.camCoordinates - fj.camCoordinates);
            if (dist2 < closeDist2) {
                // fi is too close to fj, and fj is at least as old.
                features.erase(features.begin() + i);
                break;
            }
        }
    }
}

void PointFeatureTracker::Settings::configure(const YAML::Node& node) {
    GIFeatureTracker::Settings::configure(node);
    safeConfig(node["featureDist"], featureDist);
    safeConfig(node["minHarrisQuality"], minHarrisQuality);
    safeConfig(node["maxError"], maxError);
    safeConfig(node["winSize"], winSize);
    safeConfig(node["maxLevel"], maxLevel);
    safeConfig(node["trackedFeatureDist"], trackedFeatureDist);
    safeConfig(node["equaliseImageHistogram"], equaliseImageHistogram);
    safeConfig(node["claheClipLimit"], claheClipLimit);
    safeConfig(node["claheMinMeanIntensity"], claheMinMeanIntensity);
    safeConfig(node["useFastFeatures"], useFastFeatures);

    ransacParams.maxIterations = 0; // No RANSAC by default
    safeConfig(node["ransacParams"], ransacParams);
}