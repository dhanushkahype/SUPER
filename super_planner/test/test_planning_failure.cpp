#include <gtest/gtest.h>
#include "fsm/fsm.h"
using fsm::Vec3f;
using fsm::RET_CODE;

class FailureFsm : public fsm::Fsm {
public:
    FailureFsm() {
        cfg_.stuck_give_up_count = 60;
        machine_state_ = GENERATE_TRAJ;
        gi_.goal_p = Vec3f(14.55, 0.55, 1.45);
        gi_.new_goal = true;
    }
    void fail(int count, bool maneuver = false) {
        for (int i = 0; i < count; ++i)
            recordPlanFailure("regression", maneuver, 100.0 + i);
    }
    void succeed() { onPlanSuccess(); }
    bool waiting() const { return machine_state_ == WAIT_GOAL; }
    bool finished() const { return finish_plan; }
    Vec3f goal() const { return gi_.goal_p; }
    int failures() const { return plan_fail_streak_; }
    bool remembered() const { return has_giveup_goal_; }
    void publishPolyTraj() override { FAIL() << "failure must never publish a trajectory"; }
    void publishTrajectoryPlanStatus(RET_CODE, double, bool) override {}
    void publishCurPoseToPath() override {}
    void resetVisualizedPath() override {}
};

TEST(PlanningFailure, RetryThresholdsNeverMutateGoalOrRequireMapMutation) {
    // No planner/map object is installed. All retry thresholds must be safe
    // state-only operations, including the former re-snap/relax/reset counts.
    FailureFsm f;
    const auto goal = f.goal();
    for (int i = 1; i <= 59; ++i) {
        f.fail(1);
        EXPECT_EQ(f.failures(), i);
        EXPECT_EQ((f.goal() - goal).norm(), 0.0);
        EXPECT_FALSE(f.waiting());
    }
    f.fail(1);
    EXPECT_TRUE(f.waiting());
    EXPECT_TRUE(f.finished());
    EXPECT_TRUE(f.remembered());
    EXPECT_EQ((f.goal() - goal).norm(), 0.0);
}

TEST(PlanningFailure, SuccessfulPlanResetsConsecutiveFailureBudget) {
    FailureFsm f;
    f.fail(40);
    f.succeed();
    EXPECT_EQ(f.failures(), 0);
    f.fail(59);
    EXPECT_FALSE(f.waiting());
}

TEST(PlanningFailure, ManeuverRetainsExistingWatchdogAuthority) {
    FailureFsm f;
    const auto goal = f.goal();
    f.fail(65, true);
    EXPECT_FALSE(f.waiting());
    EXPECT_EQ((f.goal() - goal).norm(), 0.0);
}
