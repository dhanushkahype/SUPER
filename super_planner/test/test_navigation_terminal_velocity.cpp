#include <gtest/gtest.h>
#include "utils/geometry/navigation_terminal_velocity.hpp"
using Eigen::Vector3d;

static Vector3d terminal(const Vector3d& position, const Vector3d& local_end,
                         const Vector3d& goal, bool stop, bool continuous = true,
                         bool goal_velocity = false) {
    return super_utils::navigationTerminalVelocity(position, local_end, goal,
        stop, continuous, goal_velocity, 1.2, 7.0, .2, 1.5);
}

TEST(NavigationTerminalVelocity, ConnectedFinalEndpointStopsBeforeDistanceGate) {
    const Vector3d goal(3, 0, 1.5), position(0, 0, 1.5);
    EXPECT_DOUBLE_EQ(terminal(position, goal, goal, true).norm(), 0.0);
    EXPECT_DOUBLE_EQ(terminal(position, goal, goal, false).norm(), .75);
}
TEST(NavigationTerminalVelocity, DistantFinalGoalDoesNotStopEveryLocalHorizon) {
    EXPECT_DOUBLE_EQ(terminal({0,0,1.5}, {7,0,1.5}, {12,0,1.5}, true).norm(), .75);
    // XY coincidence alone is not a 3D connection to a final goal.
    EXPECT_DOUBLE_EQ(terminal({0,0,1.5}, {0,0,4}, {0,0,8}, true).z(), .75);
}
TEST(NavigationTerminalVelocity, SnappedEndpointAndLegacyNearGoalStillSettle) {
    EXPECT_DOUBLE_EQ(terminal({0,0,1.5}, {3.05,.05,1.55}, {3,0,1.5}, true).norm(), 0.0);
    EXPECT_DOUBLE_EQ(terminal({2.5,0,1.5}, {3,0,1.5}, {3,0,1.5}, false).norm(), 0.0);
    EXPECT_DOUBLE_EQ(terminal({0,0,1.5}, {3,0,1.5}, {3,0,1.5}, false, false).norm(), 0.0);
}
TEST(NavigationTerminalVelocity, ExplicitStopOverridesEnabledGoalVelocity) {
    EXPECT_DOUBLE_EQ(terminal({0,0,1.5}, {10,0,1.5}, {10,0,1.5}, true, false, true).norm(), 0.0);
    EXPECT_DOUBLE_EQ(terminal({0,0,1.5}, {7,0,1.5}, {10,0,1.5}, false, false, true).norm(), .75);
}
