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

#pragma once

#include "GIFT/EgoMotion.h"

#include "GIFT/GIFeatureTracker.h"
#include "GIFT/RANSAC.h"

#include "eigen3/Eigen/Dense"

#include <memory>
#include <vector>

namespace GIFT {

class PointFeatureTracker : public GIFeatureTracker {
  protected:
    cv::Mat previousImage;
    std::vector<Feature> features;

  public:
    struct Settings : GIFeatureTracker::Settings {
        ftype featureDist = 20;
        ftype minHarrisQuality = 0.1;
        float maxError = 1e8;
        int winSize = 21;
        int maxLevel = 3;
        ftype trackedFeatureDist = 0.0;
        bool useFastFeatures = false;

        bool equaliseImageHistogram = false;
        // Contrast-limited adaptive equalisation of the tracker input; 0 disables it.
        // When set, it takes precedence over equaliseImageHistogram: full histogram
        // equalisation amplifies frame-to-frame illumination jumps and sensor noise in
        // dark scenes, while the clip limit bounds how far local contrast may stretch.
        ftype claheClipLimit = 0.0;
        // Skip CLAHE on frames whose smoothed mean intensity falls below this value
        // (0-255): amplifying contrast in a dark frame lifts sensor noise into
        // spurious corner responses. 0 applies CLAHE to every frame. The decision
        // uses an exponential moving average of the frame means (alpha 0.05, about
        // one second at 20 Hz) so the tracker does not alternate between processed
        // and unprocessed frames inside one illumination phase.
        ftype claheMinMeanIntensity = 0.0;
        double claheMeanState = -1.0; ///< EMA of frame mean intensity; -1 = uninitialised.
        // To disable RANSAC, set max iterations to 0.
        RansacParameters ransacParams;
        std::mt19937 rng = std::mt19937(0);

        virtual void configure(const YAML::Node& node) override;
    };
    Settings settings;

  public:
    // Initialisation and configuration
    PointFeatureTracker() : GIFeatureTracker() {}
    PointFeatureTracker(const std::shared_ptr<const GICamera> cameraParams) : GIFeatureTracker(cameraParams) {}
    PointFeatureTracker(const std::shared_ptr<const GICamera> cameraParams, const cv::Mat& mask)
        : GIFeatureTracker(cameraParams, mask) {}
    template <class CamClass, std::enable_if_t<std::is_base_of<GICamera, CamClass>::value, bool> = true>
    PointFeatureTracker(const CamClass& cameraParams) : GIFeatureTracker(cameraParams) {}

    // Core
    virtual void processImage(const cv::Mat& image, const std::map<int, cv::Point2f>& predictedFeatures = {});
    virtual void detectFeatures(const cv::Mat& image) override;
    virtual void trackFeatures(const cv::Mat& image, const std::map<int, cv::Point2f>& predictedFeatures);
    virtual void trackFeatures(const cv::Mat& image) override { trackFeatures(image, {}); };
    std::vector<Feature> outputFeatures() const { return features; };

    // EgoMotion
    EgoMotion computeEgoMotion(int minLifetime = 1) const;

  protected:
    std::vector<cv::Point2f> identifyFeatureCandidates(const cv::Mat& image) const;
    std::vector<cv::Point2f> removeDuplicateFeatures(const std::vector<cv::Point2f>& proposedFeatures) const;
    static void removeFeaturesTooClose(std::vector<Feature>& features, const ftype& closeDist);

    void addNewFeatures(std::vector<Feature> newFeatures);
    void computeLandmarkPositions();
};

} // namespace GIFT
