#ifndef ARMOR_DETECTOR_TOOLS_MATH_TOOLS_HPP_
#define ARMOR_DETECTOR_TOOLS_MATH_TOOLS_HPP_

// std
#include <cmath>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// opencv
#include <opencv2/core/eigen.hpp>
// tf2
#include <tf2_ros/buffer.h>

namespace pka {

inline Eigen::Vector3d rotationMatrix2RPY(const Eigen::Matrix3d& R) {
    Eigen::Quaterniond q(R);
    // cal rpy
    tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
    Eigen::Vector3d rpy;
    tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
    return rpy;
}

inline Eigen::Quaterniond rpy2Quaternion(const Eigen::Vector3d& rpy) {
    // cal sin & cos of rpy
    auto cos_r = std::cos(rpy[0] / 2);
    auto sin_r = std::sin(rpy[0] / 2);
    auto cos_y = std::cos(rpy[2] / 2);
    auto sin_y = std::sin(rpy[2] / 2);
    auto cos_p = std::cos(rpy[1] / 2);
    auto sin_p = std::sin(rpy[1] / 2);

    // cal quaternion
    auto w = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y;
    auto x = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y;
    auto y = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y;
    auto z = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y;

    return Eigen::Quaterniond(w, x, y, z);
}

inline double limit_rad(double angle) {
    while (angle > CV_PI) angle -= 2 * CV_PI;
    while (angle <= -CV_PI) angle += 2 * CV_PI;
    return angle;
}

inline double sigmoid(double x) {
    if(x > 0) {
        return 1.0 / (1.0 + exp(-x));
    } else {
        return exp(x) / (1.0 + exp(x));
    }
}

}

#endif
