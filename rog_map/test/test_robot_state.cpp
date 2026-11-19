#include <rog_map/rog_map.h>
#include <limits>
#include <iostream>
#include <stdexcept>

class StateMap : public rog_map::ROGMap {
public:
    explicit StateMap(const std::string& config) {
        cfg_ = rog_map::Config(config);
        initProbMap();
        slideAllMap(rog_map::Vec3f(0, 0, 1.5));
    }
    using ROGMap::updateRobotState;
    double time{1.0};
    const double getSystemWalltimeNow() override { return time; }
};

static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char** argv) {
    if (argc != 2) return 2;
    try {
        StateMap map(argv[1]);
        const auto initial = map.getRobotState();
        require(!initial.rcv && initial.v.isZero() && initial.a.isZero() && initial.j.isZero(),
                "missing odometry must have a deterministic, unreceived state");
        const rog_map::Vec3f position(0, 0, 1.5);
        const super_utils::Quatf yaw(Eigen::AngleAxisd(std::acos(-1.0) / 2,
                                                 rog_map::Vec3f::UnitZ()));
        require(map.updateRobotState({position, yaw}, {0.6, 0, 0}), "valid odometry rejected");
        auto state = map.getRobotState();
        require(state.rcv && state.p.isApprox(position) &&
                state.v.isApprox(rog_map::Vec3f(0, 0.6, 0), 1e-12),
                "child-frame velocity must rotate into the map frame");
        require(state.rcv_time == 1.0, "odometry receive time missing");

        // Reject corrupt samples without turning them into fresh zero velocity.
        map.time = 2.0;
        require(!map.updateRobotState({position, yaw},
                {std::numeric_limits<double>::quiet_NaN(), 0, 0}), "NaN twist accepted");
        require(!map.updateRobotState({position, super_utils::Quatf(0, 0, 0, 0)}),
                "zero quaternion accepted");
        state = map.getRobotState();
        require(state.rcv_time == 1.0 && state.v.isApprox(rog_map::Vec3f(0, 0.6, 0), 1e-12),
                "invalid odometry changed the last valid state or its freshness");

        const super_utils::Quatf pitch(Eigen::AngleAxisd(std::acos(-1.0) / 2,
                                                   rog_map::Vec3f::UnitY()));
        require(map.updateRobotState({position, pitch}, {0.6, 0, 0}) &&
                map.getRobotState().v.isApprox(rog_map::Vec3f(0, 0, -0.6), 1e-12),
                "3D body velocity rotation failed");
        require(map.updateRobotState({position, yaw}) && map.getRobotState().v.isZero(),
                "pose-only callers must have defined zero velocity");
        std::cout << "Robot state regressions passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
