/*
    This file is part of GIFT.

    GIFT is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
*/

#pragma once

#include "GIFT/camera/PinholeCamera.h"

namespace GIFT {

// Devernay--Faugeras field-of-view camera model.  The sole distortion
// parameter omega is zero only in the pinhole limit.
class FOVCamera : public PinholeCamera {
  protected:
    ftype omega;

    Eigen::Vector3T undistortPointEigen(const Eigen::Vector2T& point) const override;
    Eigen::Vector2T projectPointEigen(const Eigen::Vector3T& point) const override;

  public:
    FOVCamera(cv::Size sze = cv::Size(0, 0), cv::Mat K = cv::Mat::eye(3, 3, CV_64F), ftype omega = 0.0);
    ftype distortion() const { return omega; }
    Eigen::Matrix<ftype, 2, 3> projectionJacobian(const Eigen::Vector3T& point) const override;
};

} // namespace GIFT
