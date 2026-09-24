#pragma once

#include <algorithm>
#include <limits>

#include <super_utils/type_utils.hpp>

namespace super_planner {

inline super_utils::Vec3f nearestManeuverPoint(
    const super_utils::Vec3f &position,
    const super_utils::vec_Vec3f &path) {
    double best = std::numeric_limits<double>::infinity();
    super_utils::Vec3f nearest = position;
    for (size_t i = 1; i < path.size(); ++i) {
        const auto segment = path[i] - path[i - 1];
        const double length_squared = segment.squaredNorm();
        if (length_squared < 1e-12) continue;
        const double fraction = std::clamp(
            (position - path[i - 1]).dot(segment) / length_squared, 0.0, 1.0);
        const super_utils::Vec3f candidate = path[i - 1] + fraction * segment;
        const double distance_squared = (position - candidate).squaredNorm();
        if (distance_squared < best) {
            best = distance_squared;
            nearest = candidate;
        }
    }
    return nearest;
}

}  // namespace super_planner
