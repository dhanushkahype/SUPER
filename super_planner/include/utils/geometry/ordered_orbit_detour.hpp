#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace super_planner {

using OrbitPoints = std::vector<Eigen::Vector3d>;

// CorridorGenerator erodes its seed bounding box by robot radius in Z.
// A larger maneuver clearance therefore requires a larger search box, not a
// smaller safety margin. Preserve at least the configured free half-width.
inline double maneuverCorridorBound(double original_bound, double clearance_radius)
{
    return clearance_radius + std::max(0.5, original_bound);
}

struct OrbitProjection {
    double distance = std::numeric_limits<double>::infinity();
    double progress = 0.0;
};

inline OrbitProjection projectOrbit(const Eigen::Vector3d &point, const OrbitPoints &reference)
{
    OrbitProjection result;
    double station = 0.0;
    for (size_t i = 1; i < reference.size(); ++i) {
        const Eigen::Vector3d edge = reference[i] - reference[i - 1];
        const double length = edge.norm();
        if (length < 1e-8) continue;
        const double fraction = std::clamp((point-reference[i-1]).dot(edge)/(length*length), 0.0, 1.0);
        const double distance = (point-reference[i-1]-fraction*edge).norm();
        if (distance < result.distance) result = {distance, station + fraction*length};
        station += length;
    }
    return result;
}

// Validate an executable curve, including its stopping tail. A braking curve
// may settle with at most 2 cm of reverse projected motion, never jump backward
// to an earlier station or leave the conservative clearance/tube contract.
inline bool validateOrbitCurve(const OrbitPoints &reference, double max_deviation,
                               double duration, double sample_dt,
                               const std::function<Eigen::Vector3d(double)> &position,
                               const std::function<bool(const Eigen::Vector3d &)> &clear)
{
    if (reference.size() < 2 || !std::isfinite(duration) || duration < 0.0 ||
        !std::isfinite(sample_dt) || sample_dt <= 0.0 ||
        !std::isfinite(max_deviation) || max_deviation <= 0.0) return false;
    const int samples = std::max(1, static_cast<int>(std::ceil(duration/sample_dt)));
    double progress = -1.0;
    for (int i = 0; i <= samples; ++i) {
        const Eigen::Vector3d point = position(duration*double(i)/samples);
        if (!point.allFinite() || !clear(point)) return false;
        const auto projection = projectOrbit(point, reference);
        if (projection.distance > max_deviation || projection.progress+0.02 < progress) return false;
        progress = std::max(progress, projection.progress);
    }
    return true;
}

// Every layer is an ordered nominal station. Only horizontal normal offsets
// are allowed: this orbit-specific planner never skips a blocked station or
// changes altitude to escape the tube. The callback checks conservative map
// clearance (including sample/voxel uncertainty), not just occupancy.
inline bool orderedOrbitDetour(const Eigen::Vector3d &start, const OrbitPoints &reference,
                               size_t next_station, double max_deviation, double grid_step,
                               double edge_step, const std::function<bool(const Eigen::Vector3d &)> &clear,
                               OrbitPoints &route)
{
    route.clear();
    if (reference.size() < 2 || next_station >= reference.size() || max_deviation <= 0.0 ||
        grid_step <= 0.0 || edge_step <= 0.0 || !std::isfinite(max_deviation) ||
        !std::isfinite(grid_step) || !std::isfinite(edge_step) || !start.allFinite() || !clear(start) ||
        projectOrbit(start, reference).distance > max_deviation) return false;
    for (const auto &point : reference) if (!point.allFinite()) return false;
    struct Candidate {
        Eigen::Vector3d point;
        double offset;
        double cost = std::numeric_limits<double>::infinity();
        int previous = -1;
    };
    std::vector<std::vector<Candidate>> layers;
    layers.push_back({{start, 0.0, 0.0, -1}});
    const int cells = std::max(1, static_cast<int>(std::ceil(max_deviation/grid_step)));
    auto edge_clear = [&](const Eigen::Vector3d &from, const Eigen::Vector3d &to) {
        const Eigen::Vector3d delta = to-from;
        const int checks = std::max(1, static_cast<int>(std::ceil(delta.norm()/edge_step)));
        double progress = projectOrbit(from, reference).progress;
        for (int j = 1; j <= checks; ++j) {
            const Eigen::Vector3d point = from + delta*(double(j)/checks);
            const auto projected = projectOrbit(point, reference);
            if (projected.distance > max_deviation + 1e-8 ||
                projected.progress + 1e-5 < progress || !clear(point)) return false;
            progress = projected.progress;
        }
        return true;
    };
    // Preserve the nominal curve exactly whenever its stations and connecting
    // edges are already clear; detour cost must not shrink an unobstructed ring.
    OrbitPoints nominal{start};
    bool nominal_clear = true;
    for (size_t i = next_station; i < reference.size(); ++i) {
        if (!edge_clear(nominal.back(), reference[i])) { nominal_clear = false; break; }
        nominal.push_back(reference[i]);
    }
    if (nominal_clear) { route = std::move(nominal); return true; }
    for (size_t i = next_station; i < reference.size(); ++i) {
        const Eigen::Vector3d tangent = reference[std::min(i+1, reference.size()-1)] -
                                       reference[i ? i-1 : i];
        if (tangent.head<2>().norm() < 1e-8 || std::abs(tangent.z()) > 1e-5) return false;
        const Eigen::Vector3d normal(-tangent.y()/tangent.head<2>().norm(),
                                    tangent.x()/tangent.head<2>().norm(), 0.0);
        std::vector<Candidate> layer;
        for (int k = -cells; k <= cells; ++k) {
            const double offset = max_deviation*k/cells;
            const Eigen::Vector3d point = reference[i]+normal*offset;
            if (!clear(point)) continue;
            Candidate candidate{point, offset};
            const auto &previous = layers.back();
            for (size_t j = 0; j < previous.size(); ++j) {
                if (!std::isfinite(previous[j].cost)) continue;
                const double length = (point-previous[j].point).norm();
                const double shift = offset-previous[j].offset;
                const double cost = previous[j].cost + length + 5.0*offset*offset + 2.0*shift*shift;
                if (cost < candidate.cost && edge_clear(previous[j].point, point)) {
                    candidate.cost = cost;
                    candidate.previous = static_cast<int>(j);
                }
            }
            if (candidate.previous >= 0) layer.push_back(candidate);
        }
        if (layer.empty()) return false;
        layers.push_back(std::move(layer));
    }
    const auto &last = layers.back();
    int best = static_cast<int>(std::min_element(last.begin(), last.end(),
        [](const Candidate &a, const Candidate &b) { return a.cost < b.cost; })-last.begin());
    for (size_t i = layers.size(); i-- > 0;) {
        route.push_back(layers[i][best].point);
        best = layers[i][best].previous;
    }
    std::reverse(route.begin(), route.end());
    return route.size() == reference.size()-next_station+1;
}

}  // namespace super_planner
