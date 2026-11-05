#include <gtest/gtest.h>
#include "utils/geometry/ordered_orbit_detour.hpp"

using super_planner::OrbitPoints;
using super_planner::orderedOrbitDetour;
using super_planner::projectOrbit;

namespace {
OrbitPoints arc(double radius, double angle = 1.4)
{
    OrbitPoints points;
    for (int i = 0; i <= 14; ++i) {
        const double theta = angle*i/14;
        points.emplace_back(radius*std::cos(theta), radius*std::sin(theta), 1.5);
    }
    return points;
}

// bmich pillar occupies x,y in [2.6,3.4], z in [0,8]. The .962 m
// requirement covers .85 m physical clearance + 0.1 m voxel uncertainty
// + half a .05 m edge sample. All stations remain at flight altitude.
bool pillarClear(const Eigen::Vector3d &point)
{
    const double dx = std::max({2.6-point.x(), 0.0, point.x()-3.4});
    const double dy = std::max({2.6-point.y(), 0.0, point.y()-3.4});
    return std::hypot(dx, dy) > 0.962;
}

void checkRoute(const OrbitPoints &route, const OrbitPoints &reference)
{
    ASSERT_EQ(route.size(), reference.size());
    double progress = -1.0;
    for (size_t i = 1; i < route.size(); ++i) {
        for (int k = 0; k <= 20; ++k) {
            const Eigen::Vector3d point = route[i-1]+(route[i]-route[i-1])*(double(k)/20);
            EXPECT_TRUE(pillarClear(point));
            EXPECT_DOUBLE_EQ(point.z(), 1.5);
            const auto projection = projectOrbit(point, reference);
            EXPECT_LE(projection.distance, 0.8+1e-8);
            EXPECT_GE(projection.progress+1e-5, progress);
            progress = projection.progress;
        }
    }
}
}

TEST(OrderedOrbitDetour, ClearRingKeepsNominalStations)
{
    const auto reference = arc(2.0);
    OrbitPoints route;
    ASSERT_TRUE(orderedOrbitDetour(reference.front(), reference, 1, .8, .05, .05,
                                  pillarClear, route));
    ASSERT_EQ(route.size(), reference.size());
    for (size_t i = 0; i < route.size(); ++i) EXPECT_LT((route[i]-reference[i]).norm(), 1e-12);
}

TEST(OrderedOrbitDetour, PillarDetourRetainsEveryStationWithinTube)
{
    const auto reference = arc(3.4);
    OrbitPoints route;
    ASSERT_TRUE(orderedOrbitDetour(reference.front(), reference, 1, .8, .05, .05,
                                  pillarClear, route));
    checkRoute(route, reference);
    double departure = 0.0;
    for (size_t i = 0; i < route.size(); ++i) departure = std::max(departure, (route[i]-reference[i]).norm());
    EXPECT_GT(departure, .6);
}

TEST(OrderedOrbitDetour, WiderPillarCrossingRequiresHoldWithoutSkippingStations)
{
    const auto reference = arc(std::sqrt(18.0));
    OrbitPoints route;
    EXPECT_FALSE(orderedOrbitDetour(reference.front(), reference, 1, .8, .05, .05,
                                   pillarClear, route));
    EXPECT_TRUE(route.empty());
}

TEST(OrderedOrbitDetour, ClockwiseStationOrderAlsoDetours)
{
    auto reference = arc(3.4);
    std::reverse(reference.begin(), reference.end());
    OrbitPoints route;
    ASSERT_TRUE(orderedOrbitDetour(reference.front(), reference, 1, .8, .05, .05,
                                  pillarClear, route));
    checkRoute(route, reference);
}

TEST(OrderedOrbitDetour, UnknownBarrierCannotBeSkippedByClearEndpoints)
{
    OrbitPoints reference{{0,0,1.5}, {.5,0,1.5}, {1,0,1.5}, {1.5,0,1.5}};
    OrbitPoints route;
    const auto known = [](const Eigen::Vector3d &point) {
        return point.x() < .65 || point.x() > .85;
    };
    EXPECT_FALSE(orderedOrbitDetour(reference.front(), reference, 1, .8, .05, .05,
                                   known, route));
    EXPECT_TRUE(route.empty());
}

TEST(OrderedOrbitDetour, AircraftOutsideTubeRequiresHold)
{
    const auto reference = arc(2.0);
    OrbitPoints route;
    EXPECT_FALSE(orderedOrbitDetour(Eigen::Vector3d(0,0,1.5), reference, 1, .8, .05, .05,
                                   pillarClear, route));
}

TEST(OrbitCurveValidation, BrakingTailCanSettleWithTinyReverseMotion)
{
    const OrbitPoints reference{{0,0,1.5}, {1,0,1.5}, {2,0,1.5}};
    const auto clear = [](const Eigen::Vector3d &) { return true; };
    const auto position = [](double t) {
        return Eigen::Vector3d(t <= 1.0 ? t : 1.0-0.01*(t-1.0), 0, 1.5);
    };
    EXPECT_TRUE(super_planner::validateOrbitCurve(reference, .8, 2.0, .01, position, clear));
}

TEST(OrbitCurveValidation, BackupCannotBacktrackOrLeaveTube)
{
    const OrbitPoints reference{{0,0,1.5}, {1,0,1.5}, {2,0,1.5}};
    const auto clear = [](const Eigen::Vector3d &) { return true; };
    const auto backtrack = [](double t) {
        return Eigen::Vector3d(t <= 1.0 ? t : 1.0-0.05*(t-1.0), 0, 1.5);
    };
    EXPECT_FALSE(super_planner::validateOrbitCurve(reference, .8, 2.0, .01, backtrack, clear));
    const auto shortcut = [](double t) { return Eigen::Vector3d(t, .9*std::sin(t*M_PI/2), 1.5); };
    EXPECT_FALSE(super_planner::validateOrbitCurve(reference, .8, 2.0, .01, shortcut, clear));
}

TEST(OrbitCurveValidation, ClearanceIsVerifiedBetweenSafeEndpoints)
{
    const OrbitPoints reference{{0,0,1.5}, {1,0,1.5}, {2,0,1.5}};
    const auto occupied = [](const Eigen::Vector3d &p) { return p.x() < .95 || p.x() > 1.05; };
    const auto line = [](double t) { return Eigen::Vector3d(t, 0, 1.5); };
    EXPECT_FALSE(super_planner::validateOrbitCurve(reference, .8, 2.0, .01, line, occupied));
}

TEST(ManeuverCorridorBounds, ConservativeRadiusDoesNotInvertVerticalSeedBox)
{
    const double radius = .85+std::sqrt(3.0)*.1*.5+.05*.5;
    ASSERT_GT(radius, .8);  // This configuration inverted the old seed box.
    const double bound = super_planner::maneuverCorridorBound(.8, radius);
    const double lower = 1.5-bound+radius;
    const double upper = 1.5+bound-radius;
    EXPECT_NEAR(lower, .7, 1e-12);
    EXPECT_NEAR(upper, 2.3, 1e-12);
    EXPECT_GT(upper-lower, 1.5);
    EXPECT_GE(super_planner::maneuverCorridorBound(.1, radius)-radius, .5);
}
