#ifndef ARMOR_DETECTOR_TASKS_YOLO_HPP_
#define ARMOR_DETECTOR_TASKS_YOLO_HPP_

// std
#include <string>
#include <vector>
#include <memory>
// openvino
#include <openvino/openvino.hpp>
// files
#include <armor_detector/tasks/detect.hpp>
#include <armor_detector/tools/img_tools.hpp>
#include <armor_detector/tools/math_tools.hpp>

namespace pka {

class YOLO : public Detector {
public:
    YOLO() = default;
    YOLO(const std::string& model_path, const LightParams& light_params, const int threshold, const bool fix,
    const float conf, const float nms, const Color color);

    std::vector<Armor> detect(cv::Mat& image) override;

private:
    // fix points
    bool fixPoints(Armor& armor, cv::Mat& image);

    // inference
    ov::Core core;
    ov::CompiledModel compiled_model;
    // threshold
    float conf_threshold;
    float nms_threshold;
    // switch
    bool fix_points;
};

}

#endif