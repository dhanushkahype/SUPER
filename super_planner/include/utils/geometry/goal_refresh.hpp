#pragma once
#include <cmath>

namespace super_utils {
// A repeated position/yaw must preserve an approach or a settled hold. An
// unfinished goal in WAIT_GOAL may still be reasserted after loss/recovery.
inline bool isGoalRefresh(double position_delta, double previous_yaw,
                          double requested_yaw, bool approaching, bool settled) {
    const bool same_yaw = (std::isnan(previous_yaw) && std::isnan(requested_yaw)) ||
        (std::isfinite(previous_yaw) && std::isfinite(requested_yaw) &&
         std::abs(std::remainder(requested_yaw - previous_yaw, 2.0 * std::acos(-1.0))) < 1e-4);
    return position_delta < 1e-4 && same_yaw && (approaching || settled);
}
}
