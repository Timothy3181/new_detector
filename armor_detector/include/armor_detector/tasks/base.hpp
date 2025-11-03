#ifndef ARMOR_DETECTOR_TASKS_BASE_HPP_
#define ARMOR_DETECTOR_TASKS_BASE_HPP_

// std
#include <vector>
// opencv
#include <opencv2/opencv.hpp>
// files
#include <armor_detector/tools/types.hpp>
#include <armor_detector/tools/params.hpp>

namespace pka {

class Detector {
public:
    virtual ~Detector() = default;

    virtual std::vector<Armor> detect(cv::Mat& image) = 0;

protected:
    int threshold;
    LightParams light_params;
}; 

}

#endif