#ifndef ARMOR_DETECTOR_TOOLS_CV_TYPES_HPP_
#define ARMOR_DETECTOR_TOOLS_CV_TYPES_HPP_

// std
#include <string>
#include <algorithm>
#include <unordered_map>
// opencv
#include <opencv2/opencv.hpp>
// eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
// files
#include <rm_utils/pkaLoggerCenter.hpp>

namespace pka {

constexpr int NOT_DEFINE = -255;

enum class Color {
    RED = 0,
    BLUE = 1,
    WHITE = 2,
    PURPLE = 3
};

enum class ArmorType {
    SMALL = 0,
    LARGE = 1,
    INVALID = 2
};

enum class Symbol {
    SENTRY = 0,
    HERO_1 = 1,
    ENGINEER_2 = 2,
    INFANTRY_3 = 3,
    INFANTRY_4 = 4,
    INFANTRY_5 = 5,
    OUTPOST = 6,
    BASE = 7,
    LARGEBASE = 8,
    NEGATIVE = 9
};

const std::unordered_map<std::string, Symbol> STR2SYMBOL_MAP = {
    {"1", Symbol::HERO_1},           {"2", Symbol::ENGINEER_2}, 
    {"3", Symbol::INFANTRY_3},       {"4", Symbol::INFANTRY_4}, 
    {"5", Symbol::INFANTRY_5},       {"sentry", Symbol::SENTRY},
    {"outpost", Symbol::OUTPOST},    {"base", Symbol::BASE},
    {"large_base", Symbol::LARGEBASE}
};

const std::unordered_map<Symbol, std::string> SYMBOL2STR_MAP = {
    {Symbol::HERO_1, "1"},           {Symbol::ENGINEER_2, "2"},
    {Symbol::INFANTRY_3, "3"},       {Symbol::INFANTRY_4, "4"},
    {Symbol::INFANTRY_5, "5"},       {Symbol::SENTRY, "sentry"},
    {Symbol::OUTPOST, "outpost"},    {Symbol::BASE, "base"},
    {Symbol::LARGEBASE, "large_base"}
};

inline std::string armorType2Str(const ArmorType& type) {
    switch (type) {
        case ArmorType::SMALL:
            return "small";
        case ArmorType::LARGE:
            return "large";
        default:
            return "invalid";
    }
}

inline std::string symbol2str(const Symbol symbol) {
    // find the target
    try {
        auto result = SYMBOL2STR_MAP.at(symbol);
        return result;
    } catch (const std::out_of_range& e) {
        return "negative";
    }
}

inline Symbol str2Symbol(const std::string symbol_name) {
    // find the target
    try {
        auto result = STR2SYMBOL_MAP.at(symbol_name);
        return result;
    } catch (const std::out_of_range& e) {
        return Symbol::NEGATIVE;
    }
}

class Light : public cv::RotatedRect {
public:
    // constructor
    Light() = default;
    explicit Light(const std::vector<cv::Point>& contour);
    explicit Light(const std::vector<cv::Point>& contour, const cv::Point2f& roi_point);
    explicit Light(const cv::Point2f& top, const cv::Point2f& bottom, const Color& color);

    // elements
    cv::Point2f top, bottom, top2bottom;
    double height;
    double width;
    float tilt_angle;
    Color color;
};

struct Armor {
    // constructor
    Armor() = default;
    explicit Armor(const Light& l1, const Light& l2);
    explicit Armor(int color_idx, int symbol_idx, float conf, std::vector<cv::Point2f> key_points);

    // elements
    // basic
    Color color;
    ArmorType type;
    cv::Point2f center;
    Light left, right;
    cv::Mat pattern;
    std::string type_name;
    std::vector<cv::Point2f> points;
    // nn
    float conf;
    cv::Rect_<float> rect;
    // classify
    Symbol symbol;
    std::string symbol_name;
    // pose
    double yaw_raw;
    Eigen::Vector3d rpy_in_odom;
    Eigen::Vector3d rpy_in_gimbal;
    Eigen::Vector3d rpy_in_camera;
    Eigen::Vector3d xyz_in_odom;
    Eigen::Vector3d xyz_in_gimbal;
    Eigen::Vector3d xyz_in_camera;
};

}

#endif
