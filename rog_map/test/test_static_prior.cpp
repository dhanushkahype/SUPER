#include <rog_map/rog_map.h>
#include <iostream>
#include <stdexcept>
#include <octomap/OcTree.h>
#include <chrono>

class TestMap : public rog_map::ROGMap {
public:
    explicit TestMap(const std::string & config) {
        cfg_ = rog_map::Config(config);
        initProbMap();
        slideAllMap(rog_map::Vec3f(0, 0, 1.5));
    }
    const double getSystemWalltimeNow() override { return 1.0; }
    void apply() {
        std::lock_guard<std::mutex> lock(map_mtx_);
        applyOccupiedPrior();
    }
    void miss(const rog_map::Vec3f & p) {
        std::lock_guard<std::mutex> lock(map_mtx_);
        missPointUpdate(p, getHashIndexFromPos(p), 999);
    }
};

static void require(bool condition, const char * message) {
    if (!condition) throw std::runtime_error(message);
}

int main(int argc, char ** argv) {
    if (argc < 2) return 2;
    try {
        TestMap map(argv[1]);
        const rog_map::Vec3f obstacle(1.05, 0.05, 1.05);
        map.setOccupiedPrior({{rog_map::Vec3f(1, 0, 1), rog_map::Vec3f(1.4, 0.4, 1.4)}});
        const auto origin = map.getLocalMapOrigin();
        const auto size = map.getLocalMapSize();
        map.apply();
        require(map.isOccupied(obstacle), "prior must restore occupied geometry");
        require(map.isOccupied(rog_map::Vec3f(1.35, 0.35, 1.35)), "coarse leaf must fill all fine cells");
        require(map.isOccupiedInflate(obstacle), "prior must update inflation");
        require(!map.isOccupied(rog_map::Vec3f(0.55, 0.05, 1.05)), "prior must not raycast or extrude geometry");
        map.miss(obstacle);
        require(!map.isOccupied(obstacle), "fixture must erase raw occupancy");
        map.apply();
        require(map.isOccupied(obstacle), "blind-spot misses must not erase static prior");
        require(map.getLocalMapOrigin() == origin && map.getLocalMapSize() == size,
                "prior must not change map bounds");
        map.hardResetLocalMap(rog_map::Vec3f(0, 0, 1.5));
        require(map.isOccupied(obstacle), "local reset must preserve global static evidence");
        map.setOccupiedPrior({});
        map.miss(obstacle);
        map.apply();
        require(!map.isOccupied(obstacle), "removed prior must permit new free evidence");
        if (argc == 3) {
            octomap::OcTree tree(0.2);
            require(tree.readBinary(argv[2]), "cannot read benchmark map");
            std::vector<rog_map::ProbMap::OccupiedBox> boxes;
            for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
                if (!tree.isNodeOccupied(*it)) continue;
                const rog_map::Vec3f p(it.getX(), it.getY(), it.getZ());
                const rog_map::Vec3f h = rog_map::Vec3f::Constant(it.getSize() / 2);
                boxes.push_back({p - h, p + h});
            }
            std::cout << "Benchmark occupied leaves: " << boxes.size() << '\n';
            map.setOccupiedPrior(std::move(boxes));
            const auto start = std::chrono::steady_clock::now();
            for (int i = 0; i < 100; ++i) map.apply();
            const double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count() / 100;
            std::cout << "Mean prior application: " << ms << " ms\n";
        }
        std::cout << "Static prior regressions passed\n";
    } catch (const std::exception & e) {
        std::cerr << e.what() << '\n'; return 1;
    }
    return 0;
}
