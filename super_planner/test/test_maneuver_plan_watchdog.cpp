#include <gtest/gtest.h>
#include "utils/geometry/maneuver_plan_watchdog.hpp"

TEST(ManeuverPlanWatchdog, BriefRefusalRetainsPreviousSafePlan)
{
    super_planner::ManeuverPlanWatchdog watchdog;
    EXPECT_FALSE(watchdog.observe(false, 10.0));
    EXPECT_FALSE(watchdog.observe(false, 10.4));
    EXPECT_FALSE(watchdog.observe(true, 10.45));
    EXPECT_FALSE(watchdog.observe(false, 10.6));
}

TEST(ManeuverPlanWatchdog, SustainedRefusalRequiresHold)
{
    super_planner::ManeuverPlanWatchdog watchdog;
    EXPECT_FALSE(watchdog.observe(false, 10.0));
    EXPECT_TRUE(watchdog.observe(false, 10.5));
    EXPECT_TRUE(watchdog.observe(false, 11.0));
}

TEST(ManeuverPlanWatchdog, ExplicitNewManeuverGetsIndependentFailureBudget)
{
    super_planner::ManeuverPlanWatchdog watchdog;
    EXPECT_FALSE(watchdog.observe(false, 10.0));
    EXPECT_TRUE(watchdog.observe(false, 10.6));
    watchdog.reset();
    EXPECT_FALSE(watchdog.observe(false, 20.0));
}
