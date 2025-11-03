#include <armor_detector/armor_detector_node.hpp>

namespace pka::detector {

ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions &options) : Node("armor_detector", options) {
    PKA_INFO("armor_detector", "Start ArmorDetector");

    // node settings
    this->debug_ = this->declare_parameter("debug", true);

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
        this->estimator_ = initEstimator();
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

    // image process
    this->image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", rclcpp::SensorDataQoS(), std::bind(&ArmorDetectorNode::imageCallback, this, std::placeholders::_1));
}

std::variant<std::shared_ptr<Tradition>, std::shared_ptr<YOLO>> ArmorDetectorNode::initDetector() {
    // init params
    LightParams light_params;
    ArmorParams armor_params;

    // use yolo switch
    bool use_yolo = this->declare_parameter("use_yolo", false);

    // get light parameters from yaml file
    light_params.min_ratio = this->declare_parameter("light.min_ratio", 0.0001);
    light_params.max_ratio = this->declare_parameter("light.max_ratio", 1.0);
    light_params.max_angle = this->declare_parameter("light.max_angle", 40.0);
    light_params.color_diff_thresh = this->declare_parameter("light.color_diff_thresh", 20);

    // get armor parameters from yaml file
    if (!use_yolo) {
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
    
    if (use_yolo) {
        PKA_INFO("armor_detector", "Initialized YOLO Detector Backend");
        return std::make_shared<YOLO>(model_path.string(), light_params, threshold, fix_points, yolo_conf, nms);
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

std::shared_ptr<Estimator> ArmorDetectorNode::initEstimator() {
    // init estimator
    std::shared_ptr<Estimator> estimator;

    // get camera info
    this->cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("camera_info", rclcpp::SensorDataQoS(), [this, &estimator](sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        // optimize yaw
        bool optimize_yaw = this->declare_parameter("estimator.optimize_yaw", true);

        // search range
        double search_range = this->declare_parameter("estimator.search_range", 140.0);

        // construct
        estimator = std::make_shared<Estimator>(camera_info, optimize_yaw, search_range);

        // reset pointer
        this->cam_info_sub_.reset();
    });

    return estimator;
}

void ArmorDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    // get image time stamp
    rclcpp::Time current = img_msg->header.stamp;

    // get cv type image
    auto image = cv_bridge::toCvShare(img_msg, "rgb8")->image;

    // update tf2 relationship
    try {
        auto gimbal2odom = this->tf_helper_->lookUpGimbal2Odom(current);
        auto camera2gimbal = this->tf_helper_->lookUpCamera2Gimbal(current);

        auto R_gimbal2odom = gimbal2odom.R_matrix;
        auto R_camera2gimbal = camera2gimbal.R_matrix;
        auto t_gimbal2odom = gimbal2odom.t;
        auto t_camera2gimbal = camera2gimbal.t;

        this->estimator_->setTFRelationship(R_gimbal2odom, t_gimbal2odom, R_camera2gimbal, t_camera2gimbal);
    } catch (...) {
        PKA_ERROR("armor_detector", "Get TF2 relationship error");
    }

    // detect
    auto armors = this->detector_->detect(image);

    // classify
    this->classifier_->classify(armors);

    // estimate
    auto armors_msg = this->estimator_->estimate(armors);

    // pub armors info
    this->armors_pub_->publish(armors_msg);
}

}