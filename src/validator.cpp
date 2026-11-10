#include "runway/model.h"

#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace runway {
namespace {
bool fail(std::string* reason, const char* message) {
    if (reason != nullptr) *reason = message;
    return false;
}
}  // namespace

void validate_problem(const Problem& problem) {
    if (problem.flights.empty()) {
        throw std::invalid_argument("at least one flight is required");
    }
    const SeparationMatrix& s = problem.separation;
    if (s.aa <= 0 || s.ad <= 0 || s.da <= 0 || s.dd <= 0) {
        throw std::invalid_argument("all separations must be strictly positive");
    }
    std::unordered_set<std::string> seen;
    for (const Flight& f : problem.flights) {
        if (f.id.empty() || !seen.insert(f.id).second) {
            throw std::invalid_argument("flight IDs must be unique and nonempty");
        }
        if (f.earliest_sec < 0) {
            throw std::invalid_argument("earliest_sec cannot be negative");
        }
        if (f.runway != (f.movement == Movement::Arrival ? "RWY-ARR" : "RWY-DEP")) {
            throw std::invalid_argument("ARR must use RWY-ARR, DEP must use RWY-DEP");
        }
    }
}

bool validate_solution(const Problem& problem,
                       const Solution& solution,
                       std::string* reason) {
    try {
        validate_problem(problem);
    } catch (const std::exception& e) {
        if (reason != nullptr) *reason = e.what();
        return false;
    }
    if (solution.status != SolveStatus::Optimal &&
        solution.status != SolveStatus::Feasible) {
        return fail(reason, "solution has no feasible schedule");
    }
    if (solution.events.size() != problem.flights.size()) {
        return fail(reason, "number of scheduled flights does not match");
    }

    std::unordered_map<std::string, const Flight*> by_id;
    for (const Flight& f : problem.flights) by_id[f.id] = &f;
    std::unordered_set<std::string> seen;
    std::int64_t sum_delay = 0;

    for (std::size_t k = 0; k < solution.events.size(); ++k) {
        const ScheduledFlight& event = solution.events[k];
        const auto it = by_id.find(event.id);
        if (it == by_id.end() || !seen.insert(event.id).second) {
            return fail(reason, "unknown or duplicated flight");
        }
        const Flight& input = *it->second;
        if (event.movement != input.movement || event.runway != input.runway) {
            return fail(reason, "movement / runway assignment mismatch");
        }
        if (event.time_sec < input.earliest_sec ||
            event.delay_sec != event.time_sec - input.earliest_sec) {
            return fail(reason, "time or delay is inconsistent");
        }
        sum_delay += event.delay_sec;
        if (k == 0) continue;

        const ScheduledFlight& previous = solution.events[k - 1];
        if (event.time_sec - previous.time_sec <
            problem.separation.between(previous.movement, event.movement)) {
            return fail(reason, "global sequence AA/AD/DA/DD gap violated");
        }
    }
    if (sum_delay != solution.total_delay_sec) {
        return fail(reason, "total delay does not match the schedule");
    }

    // For each runway, independently check gaps between successive events,
    // including events separated by other-runway operations globally.
    for (std::size_t k = 0; k < solution.events.size(); ++k) {
        const ScheduledFlight& earlier = solution.events[k];
        for (std::size_t m = k + 1; m < solution.events.size(); ++m) {
            const ScheduledFlight& later = solution.events[m];
            if (earlier.runway != later.runway) continue;
            if (later.time_sec - earlier.time_sec <
                problem.separation.between(earlier.movement, later.movement)) {
                return fail(reason, "same-runway AA/DD gap violated");
            }
        }
    }
    if (reason != nullptr) reason->clear();
    return true;
}

}  // namespace runway
