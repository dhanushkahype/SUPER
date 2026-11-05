#include <gtest/gtest.h>
#include "utils/geometry/committed_plan_provenance.hpp"

TEST(CommittedPlanProvenance, CachedFutureStartCannotBeRelabelledAfterOwnershipChange)
{
    super_planner::CommittedPlanProvenance provenance;
    provenance.capture(150.0, "ORBIT session=old generation=1", 100000000000);
    provenance.capture(150.0, "ORBIT session=new generation=2", 120000000000);
    EXPECT_EQ(provenance.source(), "ORBIT session=old generation=1");
    EXPECT_EQ(provenance.created_ns(), 100000000000);
    EXPECT_TRUE(provenance.matches(150.0));
}
TEST(CommittedPlanProvenance, NewlyCommittedPlanGetsItsOwnImmutableCreationAndSource)
{
    super_planner::CommittedPlanProvenance provenance;
    provenance.capture(10.0, "GLOBAL session=a generation=1", 1000);
    provenance.capture(11.0, "ORBIT session=a generation=2", 2000);
    EXPECT_EQ(provenance.source(), "ORBIT session=a generation=2");
    EXPECT_EQ(provenance.created_ns(), 2000);
    EXPECT_FALSE(provenance.matches(10.0));
}

#include "utils/geometry/reference_goal_gate.hpp"
TEST(ReferenceGoalGate, OldGoalAndFreshlyRestampedCarrotCannotRestartAfterCancel)
{
    super_planner::ReferenceGoalGate gate;
    gate.observeGlobalGoal(100);
    EXPECT_TRUE(gate.observeOwner("GLOBAL session=a generation=2 since=200", 205));
    EXPECT_FALSE(gate.acceptsGlobalCarrot(300));
    gate.observeGlobalGoal(210);
    EXPECT_TRUE(gate.acceptsGlobalCarrot(300));
    EXPECT_FALSE(gate.acceptsGlobalCarrot(199));
}
TEST(ReferenceGoalGate, NewGoalDeliveredBeforeOwnerCallbackStillWorks)
{
    super_planner::ReferenceGoalGate gate;
    gate.observeGlobalGoal(210);
    gate.observeOwner("GLOBAL session=a generation=2 since=200", 220);
    EXPECT_TRUE(gate.acceptsGlobalCarrot(225));
    gate.observeOwner("ORBIT session=a generation=3 since=230", 240);
    gate.observeGlobalGoal(250);
    EXPECT_FALSE(gate.acceptsGlobalCarrot(260));
}
