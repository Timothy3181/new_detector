#ifndef ARMOR_DETECTOR_TASKS_ESTIMATE_HPP_
#define ARMOR_DETECTOR_TASKS_ESTIMATE_HPP_

// std
#include <vector>
// ros2
#include <sensor_msgs/msg/camera_info.hpp>
// msg
#include <rm_interfaces/msg/armor.hpp>
#include <rm_interfaces/msg/armors.hpp>
// files
#include <armor_detector/tools/types.hpp>
#include <armor_detector/tools/math_tools.hpp>

namespace pka {

constexpr double COMMON_ARMOR_HEIGHT = 56e-3;
constexpr double SMALL_ARMOR_WIDTH = 133e-3;
constexpr double LARGE_ARMOR_WIDTH = 227e-3;

const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
    {0, -SMALL_ARMOR_WIDTH / 2, COMMON_ARMOR_HEIGHT / 2},
    {0, SMALL_ARMOR_WIDTH / 2, COMMON_ARMOR_HEIGHT / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -COMMON_ARMOR_HEIGHT / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -COMMON_ARMOR_HEIGHT / 2}
};

const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
    {0, -LARGE_ARMOR_WIDTH / 2, COMMON_ARMOR_HEIGHT / 2},
    {0, LARGE_ARMOR_WIDTH / 2, COMMON_ARMOR_HEIGHT / 2},
    {0, LARGE_ARMOR_WIDTH / 2, -COMMON_ARMOR_HEIGHT / 2},
    {0, -LARGE_ARMOR_WIDTH / 2, -COMMON_ARMOR_HEIGHT / 2}
};

class Estimator {
public:
    Estimator() = default;
    Estimator(const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info, const bool& optimized_yaw, const bool solve_in_camera, const double& search_range);
    
    std::vector<rm_interfaces::msg::Armor> estimate(std::vector<Armor>& armors);

    void setTFRelationship(
        Eigen::Matrix3d& R_g2o, Eigen::Vector3d& t_g2o, Eigen::Matrix3d& R_c2g, Eigen::Vector3d& t_c2g);

private:
    // func
    float cal2CenterDist(const cv::Point2f& image_point);
    double armorReprojectionError(const Armor& armor, double yaw);
    void optimizeYaw(Armor& armor);
    // camera info
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    // frame relationships
    Eigen::Matrix3d R_gimbal2odom_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_gimbal2odom_;
    Eigen::Vector3d t_camera2gimbal_;
    // switch optimization
    bool optimize_yaw_switch;
    bool solve_in_camera;
    // optimize search range
    double search_range;
};

}

#endif