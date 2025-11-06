#include <armor_detector/tasks/detect.hpp>

namespace pka {

Tradition::Tradition(const int& threshold, const Color& color, const LightParams& light_params, const ArmorParams& armor_params) {
    this->threshold = threshold;
    this->detect_color = color;
    this->light_params = light_params;
    this->armor_params = armor_params;
}

cv::Mat Tradition::img_preprocess(cv::Mat& image) noexcept {
    // bgr2gray
    cv::Mat gray_img;
    cv::cvtColor(image, gray_img, cv::COLOR_RGB2GRAY);

    // gray2binary
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, this->threshold, 255, cv::THRESH_BINARY);

    return binary_img;
}

bool Tradition::check_light_geometry(Light& light) noexcept {
    float ratio = light.height / light.width;
    bool ratio_ok = this->light_params.min_ratio < ratio && ratio < this->light_params.max_ratio;
    bool angle_ok = light.tilt_angle < light_params.max_angle;

    return ratio_ok && angle_ok;
}

std::vector<Light> Tradition::findLightBars(cv::Mat& binary_image, cv::Mat& rgb_image) noexcept {
    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // fit lightbar
    std::vector<Light> lightbars;
    for (const auto& contour : contours) {
        if (contour.size() < 6) continue;
        
        auto light = Light(contour);
        if (check_light_geometry(light)) {
            // get color
            int sum_r = 0, sum_b = 0;
            for (const auto &point : contour) {
                sum_r += rgb_image.at<cv::Vec3b>(point.y, point.x)[0];
                sum_b += rgb_image.at<cv::Vec3b>(point.y, point.x)[2];
            }
            if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) > this->light_params.color_diff_thresh) {
                light.color = sum_r > sum_b ? Color::RED : Color::BLUE;
            }
            lightbars.emplace_back(light);
        }
    }

    // sort lightbars
    std::sort(lightbars.begin(), lightbars.end(), [](const Light &l1, const Light &l2) {
        return l1.center.x < l2.center.x;
    });

    return lightbars;
}

bool Tradition::containLight(const int i, const int j, const std::vector<Light> &lights) noexcept {
    const Light &light_1 = lights.at(i), &light_2 = lights.at(j);
    auto points = std::vector<cv::Point2f>{
        light_1.top, light_1.bottom, light_2.top, light_2.bottom
    };
    auto bounding_rect = cv::boundingRect(points);
    double avg_height = (light_1.height + light_2.height) / 2;
    double avg_width = (light_1.width + light_2.width) / 2.0;

    // only check the lights between the selection
    for (int k = i + 1; k < j; k++) {
        const Light &test_light = lights.at(k);

        // prevent the number attraction
        if (test_light.width > 2 * avg_width) continue;

        // prevent the bullet attraction
        if (test_light.height < 0.5 * avg_height) continue;

        // check if the elements are included
        if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
            bounding_rect.contains(test_light.center)) return true;
    }
    return false;
}

ArmorType Tradition::check_armor_geometry(const Light& l1, const Light& l2) noexcept {
    // height ratio
    float light_height_ratio = l1.height < l2.height ?
        l1.height / l2.height : l2.height / l1.height;
    bool height_ratio_ok = light_height_ratio > this->armor_params.min_light_ratio;

    // avg light height
    auto avg_light_height = (l1.height + l2.height) / 2;

    // center distance
    auto center_distance = cv::norm(l1.center - l2.center) / avg_light_height;
    bool center_distance_ok = (this->armor_params.min_small_center_distance <= center_distance &&
                              center_distance < this->armor_params.max_small_center_distance) ||
                              (this->armor_params.min_large_center_distance <= center_distance &&
                              center_distance < this->armor_params.max_large_center_distance);
    // angle
    cv::Point2f diff = l1.center - l2.center;
    float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
    bool angle_ok = angle < this->armor_params.max_angle;

    // total result
    bool armor_ok = height_ratio_ok && center_distance_ok && angle_ok;

    // judge armor type
    ArmorType type;
    if (armor_ok) {
        type = center_distance > this->armor_params.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL;
    } else {
        type = ArmorType::INVALID;
    }

    return type;
}

std::vector<Armor> Tradition::matchLights(std::vector<Light>& lightbars, cv::Mat& image) noexcept {
    std::vector<Armor> armors;
    // loop all the pairing of lights
    for (auto light_1 = lightbars.begin(); light_1 != lightbars.end(); light_1++) {
        // if the color is not the enemy color, pass
        if (light_1->color != this->detect_color) continue;

        // max_iter_height：计算出两根灯条可以形成一个装甲板的最大宽度
        // max_large_center_distance：大装甲板最大中心距离长宽比
        double max_iter_height = light_1->height * this->armor_params.max_large_center_distance;
        for (auto light_2 = light_1 + 1; light_2 != lightbars.end(); light_2++) {
            if (light_2->color != this->detect_color) continue;

            // containLight judge
            if (this->containLight(light_1 - lightbars.begin(), light_2 - lightbars.begin(), lightbars)) continue;

            // check whether the height is less than the biggest height
            if (light_2->center.x - light_1->center.x > max_iter_height) break;
            auto type = this->check_armor_geometry(*light_1, *light_2);
            if (type != ArmorType::INVALID) {
                auto armor = Armor(*light_1, *light_2);
                armor.type = type;
                armor.type_name = armorType2Str(armor.type);
                armor.pattern = extractNumImg(image, armor);
                armors.emplace_back(armor);
            }
        }
    }
    return armors;
}

std::vector<Armor> Tradition::detect(cv::Mat& image) {
    // 1. deal with raw image
    this->binary_img = this->img_preprocess(image);
    // 2. get lightbars
    auto lightbars = this->findLightBars(this->binary_img, image);
    // 3. match lights as armors
    return this->matchLights(lightbars, image);
}

}
