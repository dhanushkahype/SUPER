#include <gtest/gtest.h>
#include "data_structure/base/trajectory.h"

// Two continuous pieces, x(t)=t^2, y(t)=t, z(t)=1.5. The polynomial
// coefficients on the second piece are expressed in its own local time.
static geometry_utils::Trajectory source(int degree) {
    Eigen::MatrixXd first = Eigen::MatrixXd::Zero(3, degree + 1);
    first(0, degree - 2) = 1;
    first(1, degree - 1) = 1;
    first(2, degree) = 1.5;
    Eigen::MatrixXd second = first;
    second(0, degree - 1) = 2;
    second(0, degree) = 1;
    second(1, degree) = 1;
    geometry_utils::Trajectory trajectory({1.0, 1.0}, {first, second});
    trajectory.start_WT = 100.0;
    return trajectory;
}

TEST(TrajectorySlice, BoundaryRemnantsRemainValidWithoutChangingTimeOrMotion) {
    for (int degree : {5, 7}) {
        const auto trajectory = source(degree);
        for (double offset : {-5e-8, 0.0, 5e-8}) {
            for (const auto interval : {std::pair<double, double>{1.0 + offset, 1.2},
                                       {0.8, 1.0 + offset}, {0.0, 1.0 + offset}}) {
                geometry_utils::Trajectory sliced;
                ASSERT_TRUE(trajectory.getPartialTrajectoryByTime(interval.first, interval.second, sliced));
                EXPECT_NEAR(sliced.start_WT, 100.0 + interval.first, 1e-12);
                EXPECT_NEAR(sliced.getTotalDuration(), interval.second - interval.first, 1e-12);
                for (int i = 0; i < sliced.getPieceNum(); ++i) {
                    EXPECT_GT(sliced[i].getDuration(), 1e-6);
                }
                for (double fraction : {0.0, 0.5, 1.0}) {
                    const double t = fraction * sliced.getTotalDuration();
                    EXPECT_LT((sliced.getState(t) - trajectory.getState(interval.first + t)).norm(), 1e-10);
                }
            }
        }
    }
}

TEST(TrajectorySlice, InvalidOrUnpublishablyShortWindowsAreRejected) {
    const auto trajectory = source(7);
    geometry_utils::Trajectory sliced;
    EXPECT_FALSE(trajectory.getPartialTrajectoryByTime(-0.1, 0.1, sliced));
    EXPECT_FALSE(trajectory.getPartialTrajectoryByTime(1.0, 1.0, sliced));
    EXPECT_FALSE(trajectory.getPartialTrajectoryByTime(1.0, 1.0 + 5e-7, sliced));
    EXPECT_FALSE(trajectory.getPartialTrajectoryByTime(0.0, 2.1, sliced));
}
