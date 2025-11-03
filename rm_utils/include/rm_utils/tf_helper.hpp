#ifndef RM_UTILS_TF_HELPER_HPP_
#define RM_UTILS_TF_HELPER_HPP_

// std
#include <memory>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// files
#include <rm_utils/pkaLoggerCenter.hpp>

namespace pka {

struct TF2Transform {
    TF2Transform() = delete;
    TF2Transform(Eigen::Matrix3d R_mat, Eigen::Vector3d t, Eigen::Quaterniond q, bool a) : 
        R_matrix(R_mat), t(t), q(q), avaliable(a) {}

    Eigen::Matrix3d R_matrix;
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
    bool avaliable;
};

class TFHelper {
public:
    TFHelper() = delete;
    TFHelper(rclcpp::Node* node);

    TF2Transform lookUpGimbal2Odom(rclcpp::Time target_time);
    TF2Transform lookUpCamera2Gimbal(rclcpp::Time target_time);
    TF2Transform lookUpCamera2Odom(rclcpp::Time target_time);
private:
    // func
    TF2Transform transformError();
    // node
    rclcpp::Node* node;
    // tf2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    std::shared_ptr<tf2_ros::CreateTimerROS> tf2_timer_interface_;
};

}

#endif