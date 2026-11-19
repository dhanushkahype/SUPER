#pragma once
#include <Eigen/Core>

namespace super_utils {
// Intermediate pursuit targets and planning-horizon cutoffs may continue.
// An explicitly stopping target is a rest endpoint as soon as this local
// trajectory actually connects to it, regardless of the aircraft's distance.
inline Eigen::Vector3d navigationTerminalVelocity(
    const Eigen::Vector3d& position, const Eigen::Vector3d& local_endpoint,
    const Eigen::Vector3d& goal, bool terminal_stop, bool continuous_following,
    bool goal_velocity_enabled, double stop_distance, double planning_horizon,
    double endpoint_tolerance, double max_velocity)
{
    const Eigen::Vector3d delta = goal - position;
    const double distance = delta.norm();
    const bool connected = (local_endpoint - goal).norm() < endpoint_tolerance;
    if (terminal_stop && connected) return Eigen::Vector3d::Zero();
    const bool continue_motion = continuous_following
        ? distance > stop_distance
        : goal_velocity_enabled && distance > planning_horizon / 2.0 && !connected;
    if (continue_motion && distance > 1e-3) return delta / distance * (max_velocity / 2.0);
    return Eigen::Vector3d::Zero();
}
}  // namespace super_utils
