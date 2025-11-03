#ifndef ARMOR_DETECTOR_TOOLS_PARAMS_HPP_
#define ARMOR_DETECTOR_TOOLS_PARAMS_HPP_

namespace pka {

struct LightParams {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    // judge color
    int color_diff_thresh;
};

struct ArmorParams {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
};

}

#endif
