#include <rm_utils/tf_helper.hpp>

namespace pka {

TFHelper::TFHelper(rclcpp::Node* node) : node(node) {
    // init tf2 buffer
    this->tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->node->get_clock());

    // init tf2 listener
    this->tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf2_buffer_);

    // init tf2 timer interface
    this->tf2_timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(
        this->node->get_node_base_interface(), this->node->get_node_timers_interface()
    );

    // set tf2 buffer timer interface
    this->tf2_buffer_->setCreateTimerInterface(this->tf2_timer_interface_);
}

TF2Transform TFHelper::transformError() {
    return TF2Transform(Eigen::Matrix3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Quaterniond(0, 0, 0, 0), false);
}

TF2Transform TFHelper::lookUpCamera2Gimbal(rclcpp::Time target_time) {
    // tf2 looks up transform
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf2_buffer_->lookupTransform("gimbal_link", "camera_optical_frame", target_time, rclcpp::Duration::from_seconds(0.01));
    } catch(const tf2::TransformException & ex) {
        PKA_ERROR("tf2_ros", "Something wrong with look up camera to gimbal transform");
        return this->transformError();
    }

    // change ros to eigen
    // rotation
    auto msg_q = t.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3 tf_rmat(tf_q);
    Eigen::Matrix3d R_camera2gimbal;
    R_camera2gimbal << tf_rmat.getRow(0).getX(), tf_rmat.getRow(0).getY(), tf_rmat.getRow(0).getZ(),
                       tf_rmat.getRow(1).getX(), tf_rmat.getRow(1).getY(), tf_rmat.getRow(1).getZ(),
                       tf_rmat.getRow(2).getX(), tf_rmat.getRow(2).getY(), tf_rmat.getRow(2).getZ();
    Eigen::Quaterniond q_camera2gimbal(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());

    // translation
    auto msg_t = t.transform.translation;
    Eigen::Vector3d t_camera2gimbal(msg_t.x, msg_t.y, msg_t.z);

    // construct TF2Transform
    return TF2Transform(R_camera2gimbal, t_camera2gimbal, q_camera2gimbal, true);
}

TF2Transform TFHelper::lookUpGimbal2Odom(rclcpp::Time target_time) {
    // tf2 looks up transform
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf2_buffer_->lookupTransform("odom", "gimbal_link", target_time, rclcpp::Duration::from_seconds(0.01));
    } catch(const tf2::TransformException & ex) {
        PKA_ERROR("tf2_ros", "Something wrong with look up gimbal to odom transform");
        return this->transformError();
    }

    // change ros to eigen
    // rotation
    auto msg_q = t.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3 tf_rmat(tf_q);
    Eigen::Matrix3d R_gimbal2odom;
    R_gimbal2odom  <<  tf_rmat.getRow(0).getX(), tf_rmat.getRow(0).getY(), tf_rmat.getRow(0).getZ(),
                       tf_rmat.getRow(1).getX(), tf_rmat.getRow(1).getY(), tf_rmat.getRow(1).getZ(),
                       tf_rmat.getRow(2).getX(), tf_rmat.getRow(2).getY(), tf_rmat.getRow(2).getZ();
    Eigen::Quaterniond q_gimbal2odom(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());

    // translation
    auto msg_t = t.transform.translation;
    Eigen::Vector3d t_gimbal2odom(msg_t.x, msg_t.y, msg_t.z);

    // construct TF2Transform
    return TF2Transform(R_gimbal2odom, t_gimbal2odom, q_gimbal2odom, true);
}

TF2Transform TFHelper::lookUpCamera2Odom(rclcpp::Time target_time) {
    // tf2 looks up transform
    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf2_buffer_->lookupTransform("odom", "camera_optical_frame", target_time, rclcpp::Duration::from_seconds(0.01));
    } catch(const tf2::TransformException & ex) {
        PKA_ERROR("tf2_ros", "Something wrong with look up camera to odom transform");
        return this->transformError();
    }

    // change ros to eigen
    // rotation
    auto msg_q = t.transform.rotation;
    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3 tf_rmat(tf_q);
    Eigen::Matrix3d R_camera2odom;
    R_camera2odom  <<  tf_rmat.getRow(0).getX(), tf_rmat.getRow(0).getY(), tf_rmat.getRow(0).getZ(),
                       tf_rmat.getRow(1).getX(), tf_rmat.getRow(1).getY(), tf_rmat.getRow(1).getZ(),
                       tf_rmat.getRow(2).getX(), tf_rmat.getRow(2).getY(), tf_rmat.getRow(2).getZ();
    Eigen::Quaterniond q_camera2odom(tf_q.getW(), tf_q.getX(), tf_q.getY(), tf_q.getZ());

    // translation
    auto msg_t = t.transform.translation;
    Eigen::Vector3d t_camera2odom(msg_t.x, msg_t.y, msg_t.z);

    // construct TF2Transform
    return TF2Transform(R_camera2odom, t_camera2odom, q_camera2odom, true);
}

}