#ifndef RM_UTILS_MODES_HPP_
#define RM_UTILS_MODES_HPP_

// std
#include <string>

namespace pka {

enum class VisionMode {
    AUTO_AIM_RED = 0,
    AUTO_AIM_BLUE = 1,
    SMALL_RUNE_RED = 2,
    SMALL_RUNE_BLUE = 3,
    LARGE_RUNE_RED = 4,
    LARGE_RUNE_BLUE = 5
};

inline std::string visionModeToString(VisionMode mode) {
    switch (mode) {
        case VisionMode::AUTO_AIM_RED:
            return "AUTO_AIM_RED";
        case VisionMode::AUTO_AIM_BLUE:
            return "AUTO_AIM_BLUE";
        case VisionMode::SMALL_RUNE_RED:
            return "SMALL_RUNE_RED";
        case VisionMode::SMALL_RUNE_BLUE:
            return "SMALL_RUNE_BLUE";
        case VisionMode::LARGE_RUNE_RED:
            return "LARGE_RUNE_RED";
        case VisionMode::LARGE_RUNE_BLUE:
            return "LARGE_RUNE_BLUE";
        default:
            return "UNKNOWN";
    }
}

}

#endif