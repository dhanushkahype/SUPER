#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace super_planner {
// Metadata belongs to the committed plan, never to its most recent publication.
// Cached plans cannot acquire a newer owner token or creation stamp on replay.
class CommittedPlanProvenance {
public:
    void capture(double start, const std::string &source, int64_t created_ns)
    {
        if (!std::isfinite(start) || start <= 0.0 || created_ns <= 0) return;
        if (valid_ && start == start_) return;
        valid_ = true;
        start_ = start;
        source_ = source;
        created_ns_ = created_ns;
    }
    bool matches(double start) const { return valid_ && start == start_; }
    const std::string &source() const { return source_; }
    int64_t created_ns() const { return created_ns_; }
private:
    bool valid_{false};
    double start_{0.0};
    std::string source_;
    int64_t created_ns_{0};
};
}  // namespace super_planner
