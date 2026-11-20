#pragma once
#include <atomic>
#include <cstdint>

namespace super_planner {
// Observe from the ROS subscription; update from serialized planner callbacks.
// Remember trips even if braking begins and ends during a long optimization.
class BrakePlanRecovery {
public:
    enum class Action { ALLOW, BLOCK, RESTART_FROM_MEASURED };
    void observe(bool held, double now) {
        const double previous = received_.exchange(now);
        const bool was_held = held_.exchange(held);
        if ((held && !was_held) || (previous >= 0.0 && now-previous > 0.5))
            generation_.fetch_add(1);
    }
    bool blocked(double now) const {
        const double received = received_.load();
        return received >= 0.0 && (held_.load() || now < received || now-received > 0.5);
    }
    uint64_t generation() const { return generation_.load(); }
    Action update(double now, bool settled) {
        const auto current = generation();
        if (current != consumed_) { pending_ = true; consumed_ = current; }
        if (blocked(now)) { pending_ = true; return Action::BLOCK; }
        if (!pending_) return Action::ALLOW;
        if (!settled) return Action::BLOCK;
        pending_ = false;
        return Action::RESTART_FROM_MEASURED;
    }
private:
    std::atomic<bool> held_{false};
    std::atomic<double> received_{-1.0};
    std::atomic<uint64_t> generation_{0};
    uint64_t consumed_{0};
    bool pending_{false};
};
}
