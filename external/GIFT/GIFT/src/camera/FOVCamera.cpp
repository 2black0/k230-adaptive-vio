/* Field-of-view camera model, after Devernay and Faugeras. */

#include "GIFT/camera/FOVCamera.h"
#include <cmath>

using namespace GIFT;

FOVCamera::FOVCamera(cv::Size sze, cv::Mat K, ftype omega) : PinholeCamera(sze, K), omega(omega) {}

Eigen::Vector2T FOVCamera::projectPointEigen(const Eigen::Vector3T& point) const {
    const Eigen::Vector2T q = point.head<2>() / point.z();
    const ftype r = q.norm();
    if (r < 1e-10 || std::abs(omega) < 1e-10) return PinholeCamera::projectPointEigen(point);
    const ftype a = 2.0 * std::tan(omega / 2.0);
    const ftype rd = std::atan(a * r) / omega;
    Eigen::Vector3T distorted;
    distorted << q * (rd / r), 1.0;
    return PinholeCamera::projectPointEigen(distorted);
}

Eigen::Vector3T FOVCamera::undistortPointEigen(const Eigen::Vector2T& point) const {
    const Eigen::Vector3T p = PinholeCamera::undistortPointEigen(point);
    const Eigen::Vector2T q = p.head<2>() / p.z();
    const ftype rd = q.norm();
    if (rd < 1e-10 || std::abs(omega) < 1e-10) return p;
    const ftype a = 2.0 * std::tan(omega / 2.0);
    const ftype r = std::tan(omega * rd) / a;
    Eigen::Vector3T result;
    result << q * (r / rd), 1.0;
    return result;
}

Eigen::Matrix<ftype, 2, 3> FOVCamera::projectionJacobian(const Eigen::Vector3T& point) const {
    const Eigen::Vector2T q = point.head<2>() / point.z();
    const ftype r = q.norm();
    Eigen::Matrix<ftype, 2, 3> perspective;
    perspective << 1.0 / point.z(), 0.0, -point.x() / (point.z() * point.z()),
                   0.0, 1.0 / point.z(), -point.y() / (point.z() * point.z());
    Eigen::Matrix2T distortion = Eigen::Matrix2T::Identity();
    if (r >= 1e-10 && std::abs(omega) >= 1e-10) {
        const ftype a = 2.0 * std::tan(omega / 2.0);
        const ftype rd = std::atan(a * r) / omega;
        const ftype drd = a / (omega * (1.0 + a * a * r * r));
        const ftype h = rd / r;
        const ftype hp_over_r = (r * drd - rd) / (r * r * r);
        distortion = h * Eigen::Matrix2T::Identity() + hp_over_r * q * q.transpose();
    }
    Eigen::Matrix2T pixels;
    pixels << fx, 0.0, 0.0, fy;
    return pixels * distortion * perspective;
}
