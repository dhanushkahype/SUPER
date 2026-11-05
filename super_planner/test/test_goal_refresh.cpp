#include <gtest/gtest.h>
#include <limits>
#include "utils/geometry/goal_refresh.hpp"

TEST(GoalRefresh, RepeatedEndpointPreservesApproachAndSettledHold) {
    const double disabled = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(super_utils::isGoalRefresh(0, disabled, disabled, true, false));
    EXPECT_TRUE(super_utils::isGoalRefresh(0, disabled, disabled, false, true));
    EXPECT_FALSE(super_utils::isGoalRefresh(0, disabled, disabled, false, false));
}
TEST(GoalRefresh, NewPositionOrYawStillCommandsMotion) {
    EXPECT_FALSE(super_utils::isGoalRefresh(0.05, 0, 0, true, false));
    EXPECT_FALSE(super_utils::isGoalRefresh(0, 0, 0.1, true, false));
    EXPECT_TRUE(super_utils::isGoalRefresh(0, 0, 2 * std::acos(-1.0), true, false));
    EXPECT_FALSE(super_utils::isGoalRefresh(0, 0, std::numeric_limits<double>::quiet_NaN(), true, false));
}
