#ifndef ARMOR_DETECTOR_ARMOR_DETECTOR_NODE_HPP_
#define ARMOR_DETECTOR_ARMOR_DETECTOR_NODE_HPP_

// std
#include <memory>
#include <string>
#include <vector>
#include <variant>
#include <functional>
// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
// msg
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rm_interfaces/msg/armors.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// srv
#include <rm_interfaces/srv/set_mode.hpp>
// tools
#include <rm_utils/modes.hpp>
#include <rm_utils/heartbeat.hpp>
#include <rm_utils/tf_helper.hpp>
#include <rm_utils/pkaLoggerCenter.hpp>
#include <rm_utils/url_resolver.hpp>
#include <armor_detector/tools/types.hpp>
#include <armor_detector/tools/params.hpp>
#include <armor_detector/tools/img_tools.hpp>
// tasks
#include <armor_detector/tasks/detect.hpp>
#include <armor_detector/tasks/classify.hpp>
#include <armor_detector/tasks/estimate.hpp>
#include <armor_detector/tasks/yolo.hpp>

namespace pka::detector {

class ArmorDetectorNode : public rclcpp::Node {
public:
    ArmorDetectorNode(const rclcpp::NodeOptions &options);

private:
    // init detector backend
    std::variant<std::shared_ptr<Tradition>, std::shared_ptr<YOLO>> initDetector();
    struct SortBackend {
        std::shared_ptr<Detector> operator()(const std::shared_ptr<Detector> detector) const { return detector; }
        std::shared_ptr<Detector> operator()(const std::shared_ptr<YOLO> yolo) const { return yolo; }
    };

    // init classifier backend
    std::shared_ptr<Classifier> initClassifier();

    // init estimator backend
    void initEstimator();

    // image process callback func
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);

    // set mode callback
    void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request, std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

    // create debug publishers
    void createDebugPub();

    // tasks
    std::shared_ptr<Detector> detector_;
    std::shared_ptr<Classifier> classifier_;
    std::shared_ptr<Estimator> estimator_;

    // tools
    std::shared_ptr<TFHelper> tf_helper_;
    HeartBeatPublisher::SharedPtr heartbeat_;

    // publishers
    rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr armors_pub_;
    visualization_msgs::msg::Marker armor_marker_;
    visualization_msgs::msg::MarkerArray armor_marker_array_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    image_transport::Publisher binary_pub_;
    image_transport::Publisher result_pub_;

    // subscribers
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_;

    // service client
    rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

    // settings
    bool debug_;
    bool use_yolo_;
};

}

#endif