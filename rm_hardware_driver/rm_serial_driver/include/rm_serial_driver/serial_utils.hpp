#ifndef RM_SERIAL_DRIVER_SERIAL_UTILS_HPP_
#define RM_SERIAL_DRIVER_SERIAL_UTILS_HPP_

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

namespace pka {

inline Eigen::Vector3d getRPY(const Eigen::Matrix3d &R) {
  double yaw = atan2(R(0, 1), R(0, 0));
  double c2 = Eigen::Vector2d(R(2, 2), R(1, 2)).norm();
  double pitch = atan2(-R(0, 2), c2);

  double s1 = sin(yaw);
  double c1 = cos(yaw);
  double roll = atan2(s1 * R(2, 0) - c1 * R(2, 1), c1 * R(1, 1) - s1 * R(1, 0));

  return -Eigen::Vector3d(roll, pitch, yaw);
}

}

#endif
