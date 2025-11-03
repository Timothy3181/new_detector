#ifndef ARMOR_DETECTOR_TASK_TRADITION_HPP_
#define ARMOR_DETECTOR_TASK_TRADITION_HPP_

// std
#include <vector>
#include <numeric>
// files
#include <armor_detector/tasks/base.hpp>
#include <armor_detector/tools/img_tools.hpp>

namespace pka {

class Tradition : public Detector {
public:
    Tradition() = default;
    Tradition(LightParams& light_params, int threshold);
    Tradition(const int& threshold, const Color& color, const LightParams& light_params, const ArmorParams& armor_params);
    
    std::vector<Armor> detect(cv::Mat& image) override;

private:
    cv::Mat img_preprocess(cv::Mat& image) noexcept;
    std::vector<Light> findLightBars(cv::Mat& binary_image, cv::Mat& rgb_image) noexcept;
    std::vector<Armor> matchLights(std::vector<Light>& lightbars, cv::Mat& image) noexcept;

    bool containLight(const int i, const int j, const std::vector<Light> &lights) noexcept;

    bool check_light_geometry(Light& light) noexcept;
    ArmorType check_armor_geometry(const Light& l1, const Light& l2) noexcept;
    
    Color detect_color;
    ArmorParams armor_params;
};

}

#endif
