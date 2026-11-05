#pragma once

namespace super_planner {

// A failed replan may retain the preceding safe plan briefly, but a maneuver
// must hold when no replacement can be committed for a bounded interval.
class ManeuverPlanWatchdog {
public:
    explicit ManeuverPlanWatchdog(double timeout = 0.5) : timeout_(timeout) {}

    void reset() { failed_since_ = -1.0; }

    bool observe(bool committed, double now)
    {
        if (committed) {
            reset();
            return false;
        }
        if (failed_since_ < 0.0) failed_since_ = now;
        return now - failed_since_ >= timeout_;
    }

private:
    double timeout_;
    double failed_since_{-1.0};
};

}  // namespace super_planner
