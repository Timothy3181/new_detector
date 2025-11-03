#ifndef ARMOR_DETECTOR_TASKS_CLASSIFY_HPP_
#define ARMOR_DETECTOR_TASKS_CLASSIFY_HPP_

// std
#include <string>
#include <vector>
#include <fstream>
#include <mutex>
// opencv
#include <opencv2/opencv.hpp>
// files
#include <armor_detector/tools/types.hpp>

namespace pka {

class Classifier {
public:
    Classifier() = delete;
    Classifier(
        const std::string& model_path, 
        const std::string& label_path, 
        const double conf, 
        const std::vector<std::string>& ignore_classes
    );

    void classify(std::vector<Armor>& armors);

private:
    // func
    void eraseIgnoreClasses(std::vector<Armor>& armors);
    // data
    std::mutex mutex_;
    double conf;
    std::vector<std::string> ignore_classes;
    std::vector<std::string> class_names_;
    cv::dnn::Net net_;
};

}

#endif