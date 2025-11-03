#ifndef ARMOR_DETECTOR_TOOLS_IMG_TOOLS_HPP_
#define ARMOR_DETECTOR_TOOLS_IMG_TOOLS_HPP_

// std
#include <vector>
#include <string>
#include <cmath>
// opencv
#include <opencv2/opencv.hpp>
// files
#include <armor_detector/tools/types.hpp>

namespace pka {

inline void drawArmor(const cv::Mat& src, const Armor& armor) {
    // draw lines
    for (int i = 0; i < 3; i++) {
        cv::line(src, armor.points[i], armor.points[i + 1], cv::Scalar(0, 255, 0), 1, 8);
    }
}

inline void drawPoint(const cv::Mat& src, const cv::Point2f& p) {
    cv::circle(src, p, 1, cv::Scalar(0, 255, 0));
}

inline void drawPoints(const cv::Mat& src, const std::vector<cv::Point>& ps) {
    for (const auto& p : ps) {
        drawPoint(src, p);
    }
}

inline void drawLine(const cv::Mat& src, const cv::Point2f& p1, const cv::Point2f& p2) {
    cv::line(src, p1, p2, cv::Scalar(0, 255, 0), 1, 8);
}

inline void drawText(const cv::Mat& src, const std::string& text, cv::Point pos, int font_face) {
    cv::putText(src, text, pos, font_face, cv::FONT_HERSHEY_SIMPLEX, cv::Scalar(0, 255, 0));
}

inline cv::Mat resizeImg(const cv::Mat& raw, double& scale_) {
    // init preprocess image
    cv::Mat image = raw;

    // get origin image scale
    auto x_s = static_cast<double>(640) / image.rows;
    auto y_s = static_cast<double>(640) / image.cols;
    auto scale = std::min(x_s, y_s);
    scale_ = scale;
    auto new_h = static_cast<int>(image.rows * scale);
    auto new_w = static_cast<int>(image.cols * scale);

    // get resized
    cv::Mat result(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Rect roi(0, 0, new_w, new_h);
    cv::resize(image, result(roi), {new_w, new_h});

    return result;
}

inline cv::Mat extractNumImg(const cv::Mat& src, const Armor& armor) {
    // Light length in image
    // 图像中的灯条长度
    static const int light_length = 12;
    // Image size after warp
    // 变换后的图像大小
    static const int warp_height = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    // Number ROI size
    // ROI区域的大小
    static const cv::Size roi_size(20, 28);
    // 输入图像的大小
    static const cv::Size input_size(28, 28);

    // Warp perspective transform
    // 透视变换


    // 提取灯条的四个点
    cv::Point2f lights_vertices[4] = {
        armor.left.bottom, armor.left.top, armor.right.top, armor.right.bottom};

    // 
    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = 
    {
        cv::Point(0, bottom_light_y),
        cv::Point(0, top_light_y),
        cv::Point(warp_width - 1, top_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
    };
    // 数字图像
    cv::Mat number_image;
    // 旋转矩阵
    // getPerspectiveTransform函数通过原图像lights_vertices和结果图像target_vertices的四个顶点坐标，从四对对应点计算透视变换。
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    // 透视变换
    // warp_width为变换的宽度，warp_height为变换的高度
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    // 获取ROI区域
    number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    // 二值化
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    // cv::THRESH_OTSU为自适应二值化
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // 对图像进行缩放
    cv::resize(number_image, number_image, input_size);
    return number_image;
}

}

#endif