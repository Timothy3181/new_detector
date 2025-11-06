#include <armor_detector/armor_detector_node.hpp>

namespace pka::detector {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("armor_detector", options) {
    PKA_INFO("armor_detector", "Start ArmorDetector");

    // node settings
    this->debug_ = this->declare_parameter("debug", true);

    // use yolo switch
    this->use_yolo_ = this->declare_parameter("use_yolo", false);

    // debug process
    if (this->debug_) {
        this->createDebugPub();
    }

    // init detector
    try {
        this->detector_ = std::visit(SortBackend(), this->initDetector());
    } catch (...) {
        PKA_ERROR("armor_detector", "Initialize Detector Error");
    }

    // init classifier
    try {
        this->classifier_ = initClassifier();
    } catch(...) {
        PKA_ERROR("armor_detector", "Initialize Classifier Error");
    }

    // init estimator
    try {
        this->initEstimator();
    } catch (...) {
        PKA_ERROR("armor_detector", "Initialize Estimator Error");
    }

    // init tf helper
    try {
        this->tf_helper_ = std::make_shared<TFHelper>(this);
    } catch (...) {
        PKA_ERROR("armor_detector", "Initialize TFHelper Error");
    }

    // armor publisher
    this->armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>("armor_detector/armors", rclcpp::SensorDataQoS());

    // init armor marker publisher
    try {
        // init marker
        this->armor_marker_.ns = "armors";
        this->armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        this->armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        this->armor_marker_.scale.x = 0.03;
        this->armor_marker_.scale.y = 0.15;
        this->armor_marker_.scale.z = 0.12;
        this->armor_marker_.color.a = 1.0;
        this->armor_marker_.color.r = 1.0;
        this->armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        // init marker publisher
        this->markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_detector/marker", 10);
    } catch (...) {
        PKA_ERROR("armor_detector", "Initialize Armor Marker Error");
    }

    // image process
    this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));

    // set mode
    this->set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>("armor_detector/set_mode", std::bind(&ArmorDetectorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

    // heartbeat
    this->heartbeat_ = HeartBeatPublisher::create(this);
}

std::variant<std::shared_ptr<Tradition>, std::shared_ptr<YOLO>> ArmorDetectorNode::initDetector() {
    // init params
    LightParams light_params;
    ArmorParams armor_params;

    // get light parameters from yaml file
    light_params.min_ratio = this->declare_parameter("light.min_ratio", 0.0001);
    light_params.max_ratio = this->declare_parameter("light.max_ratio", 8.0);
    light_params.max_angle = this->declare_parameter("light.max_angle", 40.0);
    light_params.color_diff_thresh = this->declare_parameter("light.color_diff_thresh", 20);

    // get armor parameters from yaml file
    if (!this->use_yolo_) {
        armor_params.min_small_center_distance = this->declare_parameter("armor.min_small_center_distance", 0.8);
        armor_params.max_small_center_distance = this->declare_parameter("armor.max_small_center_distance", 3.5);
        armor_params.min_large_center_distance = this->declare_parameter("armor.min_large_center_distance", 3.5);
        armor_params.max_large_center_distance = this->declare_parameter("armor.max_large_center_distance", 8.0);
        armor_params.max_angle = this->declare_parameter("armor.max_angle", 35.0);
        armor_params.min_light_ratio = this->declare_parameter("armor.min_light_ratio", 0.8);
    }

    // get binary threshold
    int threshold = this->declare_parameter("binary_threshold", 100);

    // get yolo parameters from yaml file
    auto model_path = URLResolver::getResolvedPath("package://armor_detector/model/0526.onnx");

    // fix points switch
    bool fix_points = this->declare_parameter("yolo.fix_points", true);

    // nms threshold
    float nms = this->declare_parameter("yolo.nms_threshold", 0.45);

    // yolo detect confidence
    float yolo_conf = this->declare_parameter("yolo.confidence", 0.7);
    
    if (this->use_yolo_) {
        PKA_INFO("armor_detector", "Initialized YOLO Detector Backend");
        return std::make_shared<YOLO>(model_path.string(), light_params, threshold, fix_points, yolo_conf, nms, Color::RED);
    } else {
        PKA_INFO("armor_detector", "Initialized Traditional Detector Backend");
        return std::make_shared<Tradition>(threshold, Color::RED, light_params, armor_params);
    }
}

std::shared_ptr<Classifier> ArmorDetectorNode::initClassifier() {
    // get data
    auto model_path = URLResolver::getResolvedPath("package://armor_detector/model/lenet.onnx");
    auto label_path = URLResolver::getResolvedPath("package://armor_detector/model/label.txt");
    double threshold = this->declare_parameter("classifier.confidence", 0.7);
    std::vector<std::string> ignore_classes = this->declare_parameter("ignore_classes", std::vector<std::string>{"negative"});

    return std::make_shared<Classifier>(model_path.string(), label_path.string(), threshold, ignore_classes);
}

void ArmorDetectorNode::initEstimator() {
    // get camera info
    this->cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        // storage camera info
        this->camera_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);

        // optimize yaw
        bool optimize_yaw = this->declare_parameter("estimator.optimize_yaw", true);

        // search range
        double search_range = this->declare_parameter("estimator.search_range", 140.0);

        // construct
        this->estimator_ = std::make_shared<Estimator>(this->camera_info_, optimize_yaw, search_range);

        // reset pointer
        this->cam_info_sub_.reset();
    });
}

void ArmorDetectorNode::createDebugPub() {
    this->binary_pub_ = image_transport::create_publisher(this, "armor_detector/binary_img");
    this->result_pub_ = image_transport::create_publisher(this, "armor_detector/result_img");
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    // check if estimator is empty
    if (this->estimator_ == nullptr) {
        PKA_ERROR("armor_detector", "Estimator hasn't initialized yet");
        return;
    }

    try {
        // get current time stamp
        rclcpp::Time current = img_msg->header.stamp;

        // update tf2 relationship
        auto gimbal2odom = this->tf_helper_->lookUpGimbal2Odom(current);
        auto camera2gimbal = this->tf_helper_->lookUpCamera2Gimbal(current);

        auto R_gimbal2odom = gimbal2odom.R_matrix;
        auto R_camera2gimbal = camera2gimbal.R_matrix;
        auto t_gimbal2odom = gimbal2odom.t;
        auto t_camera2gimbal = camera2gimbal.t;

        this->estimator_->setTFRelationship(R_gimbal2odom, t_gimbal2odom,R_camera2gimbal, t_camera2gimbal);
    } catch (...) {
        return;
    }

    // get cv type image
    auto image = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    // detect
    auto armors = this->detector_->detect(image);

    // classify
    if (this->use_yolo_) this->classifier_->classify(armors);

    // estimate
    auto armors_msg_vec = this->estimator_->estimate(armors);

    // mark & build armors msg
    this->armor_marker_array_.markers.clear();
    this->armor_marker_.header.stamp = img_msg->header.stamp;
    this->armor_marker_.id = 0;
    rm_interfaces::msg::Armors armors_msg;
    for (auto& armor_msg : armors_msg_vec) {
        this->armor_marker_.pose = armor_msg.pose;
        this->armor_marker_.id++;
        this->armor_marker_array_.markers.emplace_back(this->armor_marker_);
        armors_msg.data.emplace_back(armor_msg);
    }
    this->armor_marker_.action = armors_msg.data.empty() ? visualization_msgs::msg::Marker::DELETEALL : visualization_msgs::msg::Marker::ADD;
    armor_marker_array_.markers.emplace_back(this->armor_marker_);
    this->markers_pub_->publish(armor_marker_array_);

    // publish result image
    if (this->debug_) {
        cv::Mat result_img = image.clone();
        drawArmors(result_img, armors);
        this->binary_pub_.publish(cv_bridge::CvImage(img_msg->header, "mono8", this->detector_->binary_img).toImageMsg());
        this->result_pub_.publish(cv_bridge::CvImage(img_msg->header, "rgb8", result_img).toImageMsg());
    }

    // pub armors info
    this->armors_pub_->publish(armors_msg);
}

void ArmorDetectorNode::setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request, std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
    // set response
    response->success = true;
    response->message = "0";

    // get mode
    auto mode = static_cast<VisionMode>(request->mode);
    auto mode_name = visionModeToString(mode);
    if (mode_name == "UNKNOWN") {
        PKA_ERROR("armor_detector", "Set mode error");
        return;
    }

    // reset image sub
    auto createImageSub = [this]() {
        if (this->image_sub_ == nullptr) {
            this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
        }
    };

    // switch detect mode
    switch (mode) {
        case VisionMode::AUTO_AIM_RED:
            this->detector_->setDetectColor(Color::RED);
            createImageSub();
            PKA_WARN("armor_detector", "Detect color switch to red");
            break;
        case VisionMode::AUTO_AIM_BLUE:
            this->detector_->setDetectColor(Color::BLUE);
            createImageSub();
            PKA_WARN("armor_detector", "Detect color switch to blue");
            break;
        default:
            this->image_sub_.reset();
    }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pka::detector::ArmorDetectorNode)