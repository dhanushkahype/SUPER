#include <gtest/gtest.h>
#include "utils/geometry/brake_plan_recovery.hpp"
using Recovery = super_planner::BrakePlanRecovery;
using Action = Recovery::Action;
TEST(BrakePlanRecovery, HoldsUntilClearAndSettledThenRestartsOnce) {
    Recovery r;
    r.observe(false, 1.0);
    EXPECT_EQ(r.update(1.1, true), Action::ALLOW);
    r.observe(true, 1.2);
    EXPECT_EQ(r.update(1.3, true), Action::BLOCK);
    r.observe(false, 1.4);
    EXPECT_EQ(r.update(1.5, false), Action::BLOCK);
    EXPECT_EQ(r.update(1.6, true), Action::RESTART_FROM_MEASURED);
    EXPECT_EQ(r.update(1.7, true), Action::ALLOW);
}
TEST(BrakePlanRecovery, TripDuringOptimizationInvalidatesOldGeneration) {
    Recovery r;
    r.observe(false, 1.0);
    const auto plan = r.generation();
    r.observe(true, 1.1); r.observe(false, 1.2);
    EXPECT_NE(r.generation(), plan);
    EXPECT_EQ(r.update(1.3, true), Action::RESTART_FROM_MEASURED);
}
TEST(BrakePlanRecovery, LostSourceFailsClosedAndRequiresMeasuredRestart) {
    Recovery r;
    r.observe(false, 1.0);
    EXPECT_EQ(r.update(1.6, true), Action::BLOCK);
    r.observe(false, 1.7);
    EXPECT_EQ(r.update(1.8, true), Action::RESTART_FROM_MEASURED);
    EXPECT_EQ(r.update(1.9, true), Action::ALLOW);
}
