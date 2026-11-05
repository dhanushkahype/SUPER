#pragma once

#include <algorithm>
#include <cstdint>
#include <string>

namespace super_planner {
// An ownership change revokes the old global goal. A new global request must
// be causally newer than the mux transition before its pursuit carrots can
// restart planning. The mux timestamp tolerates cross-topic delivery reorder.
class ReferenceGoalGate {
public:
    bool observeOwner(const std::string &owner, int64_t received_ns)
    {
        if (owner == owner_) return false;
        owner_ = owner;
        cutoff_ns_ = received_ns;
        const auto key = owner.find(" since=");
        if (key != std::string::npos) {
            try { cutoff_ns_ = std::stoll(owner.substr(key+7)); }
            catch (...) { cutoff_ns_ = received_ns; }
        }
        return true;
    }
    void observeGlobalGoal(int64_t stamp_ns) { latest_goal_ns_ = std::max(latest_goal_ns_, stamp_ns); }
    bool acceptsGlobalCarrot(int64_t stamp_ns) const
    {
        return owner_.rfind("GLOBAL", 0) == 0 && latest_goal_ns_ > cutoff_ns_ && stamp_ns > cutoff_ns_;
    }
    const std::string &owner() const { return owner_; }
private:
    std::string owner_{"GLOBAL session= generation=0 since=0"};
    int64_t cutoff_ns_{0};
    int64_t latest_goal_ns_{0};
};
}  // namespace super_planner
