#include <armor_detector/tools/types.hpp>

namespace pka {

// 传统灯条构造函数
Light::Light(const std::vector<cv::Point>& contour) : cv::RotatedRect(cv::minAreaRect(contour)) {
    PKA_ASSERT(contour.size() > 0);

    // calculate center
    this->center = std::accumulate(
        contour.begin(),
        contour.end(),
        cv::Point2f(0, 0),
        [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
            return a + cv::Point2f(b.x, b.y) / n;
        }
    );

    // store four corners
    cv::Point2f p[4];
    this->points(p);
    std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    // cal pos
    this->top = (p[0] + p[1]) / 2;
    this->bottom = (p[2] + p[3]) / 2;
    this->height = cv::norm(this->top - this->bottom);
    this->width = cv::norm(p[0] - p[1]);

    // cal top2bottom
    this->top2bottom = this->bottom - this->top;

    // tilt angle
    this->tilt_angle = std::atan2(std::abs(this->top2bottom.x), std::abs(this->top2bottom.y));
    this->tilt_angle = this->tilt_angle / CV_PI * 180.0;
}

// YOLO传统修正灯条构造函数
Light::Light(const std::vector<cv::Point>& contour, const cv::Point2f& roi_point) : cv::RotatedRect(cv::minAreaRect(contour)) {
    PKA_ASSERT(contour.size() > 0);

    // calculate center
    this->center = std::accumulate(
        contour.begin(),
        contour.end(),
        cv::Point2f(0, 0),
        [n = static_cast<float>(contour.size())](const cv::Point2f& a, const cv::Point& b) {
            return a + cv::Point2f(b.x, b.y) / n;
        }
    ) + roi_point;

    // store four corners
    cv::Point2f p[4];
    this->points(p);
    std::sort(p, p + 4, [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    // cal pos
    this->top = ((p[0] + roi_point) + (p[1] + roi_point)) / 2;
    this->bottom = ((p[2] + roi_point) + (p[3] + roi_point)) / 2;
    this->height = cv::norm(this->top - this->bottom);
    this->width = cv::norm((p[0] + roi_point) - (p[1] + roi_point));

    // cal top2bottom
    this->top2bottom = this->bottom - this->top;

    // tilt angle
    this->tilt_angle = std::atan2(std::abs(this->top2bottom.x), std::abs(this->top2bottom.y));
    this->tilt_angle = this->tilt_angle / CV_PI * 180.0;
}

// YOLO灯条构造函数
Light::Light(const cv::Point2f& top, const cv::Point2f& bottom, const Color& color) : top(top), bottom(bottom), color(color) {
    // cal top2bottom
    this->top2bottom = this->bottom - this->top;

    // cal height & width
    this->height = cv::norm(this->top2bottom);
    this->width = NOT_DEFINE;

    // cal center
    this->center = (this->top + this->bottom) / 2;

    // tilt angle
    this->tilt_angle = std::atan2(std::abs(this->top2bottom.x), std::abs(this->top2bottom.y));
    this->tilt_angle = this->tilt_angle / CV_PI * 180.0;
}

// 传统识别装甲板构造函数
Armor::Armor(const Light& l1, const Light& l2) {
    // set lights
    if (l1.center.x < l2.center.x) {
        this->left = l1;
        this->right = l2;
    } else {
        this->left = l2;
        this->right = l1;
    }

    // 顺时针方向
    this->points = {
        this->left.top, this->right.top, this->right.bottom, this->left.bottom
    };

    // cal center
    this->center = (this->left.center + this->right.center) / 2;

    // color
    if (this->left.color == this->right.color) this->color = this->left.color;
}

// YOLO装甲板构造函数
Armor::Armor(int color_idx, int symbol_idx, float conf, std::vector<cv::Point2f> key_points) {
    // base
    this->color = static_cast<Color>(color_idx);
    this->symbol = static_cast<Symbol>(symbol_idx);
    this->symbol_name = symbol2str(this->symbol);
    this->conf = conf;

    // points
    this->points = key_points;

    // lights
    this->left = Light(key_points[0], key_points[3], this->color);
    this->right = Light(key_points[1], key_points[2], this->color);

    // center
    this->center = (key_points[0] + key_points[1] + key_points[2] + key_points[3]) / 4;

    // armor type
    if (this->symbol == Symbol::HERO_1 || this->symbol == Symbol::BASE || this->symbol == Symbol::LARGEBASE) {
        this->type = ArmorType::LARGE;
    } else {
        this->type = ArmorType::SMALL;
    }
    this->type_name = armorType2Str(this->type);
}

}
