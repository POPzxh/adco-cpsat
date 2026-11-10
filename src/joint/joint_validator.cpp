#include "runway/joint.h"

#include <algorithm>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace runway {
namespace {

bool in_windows(std::int64_t time, const std::vector<TimeWindow>& windows) {
    if (windows.empty()) return true;
    for (const TimeWindow& w : windows) {
        if (w.start_sec <= time && time <= w.end_sec) return true;
    }
    return false;
}

void check_windows(const std::vector<TimeWindow>& windows,
                   std::int64_t horizon) {
    for (const TimeWindow& w : windows) {
        if (w.start_sec < 0 || w.start_sec > w.end_sec ||
            w.end_sec > horizon) {
            throw std::invalid_argument("invalid time window");
        }
    }
}

bool fail(std::string* reason, const std::string& message) {
    if (reason) *reason = message;
    return false;
}

struct TimedEvent {
    std::string id;
    Movement movement;
    std::int64_t time;
};

bool by_time(const TimedEvent& a, const TimedEvent& b) {
    if (a.time != b.time) return a.time < b.time;
    return a.id < b.id;
}

} // namespace

void validate_joint_problem(const JointProblem& p) {
    if (p.horizon_sec <= 0 || p.horizon_sec > 10000000 || p.flights.empty()) {
        throw std::invalid_argument("invalid horizon or empty problem");
    }
    if (p.runway_separation.aa <= 0 || p.runway_separation.ad <= 0 ||
        p.runway_separation.da <= 0 || p.runway_separation.dd <= 0 ||
        p.fix_gap_sec <= 0) {
        throw std::invalid_argument("separations must be positive");
    }
    std::set<std::string> runways(p.runways.begin(), p.runways.end());
    std::set<std::string> fixes(p.fixes.begin(), p.fixes.end());
    if (runways.empty() || runways.size() != p.runways.size() ||
        fixes.size() != p.fixes.size() ||
        runways.count("") || fixes.count("")) {
        throw std::invalid_argument("resource names must be unique and nonempty");
    }
    std::map<std::string, const JointFlight*> flights;
    for (const JointFlight& f : p.flights) {
        if (f.id.empty() || !flights.emplace(f.id, &f).second ||
            f.input_time_sec < 0 || f.input_time_sec > p.horizon_sec ||
            f.reference_sec < 0 || f.reference_sec > p.horizon_sec ||
            f.options.empty()) {
            throw std::invalid_argument("invalid flight identity, time or options");
        }
        check_windows(f.hard_operation_windows, p.horizon_sec);
        std::set<std::string> ids;
        for (const FlightOption& o : f.options) {
            if (o.id.empty() || !ids.insert(o.id).second ||
                !runways.count(o.runway)) {
                throw std::invalid_argument("invalid or duplicate option / runway");
            }
            check_windows(o.operation_windows, p.horizon_sec);
            if (f.movement == Movement::Arrival) {
                if (o.star.empty() || !fixes.count(o.fix) ||
                    o.entry_to_fix_min_sec < 0 ||
                    o.entry_to_fix_max_sec < o.entry_to_fix_min_sec ||
                    o.fix_to_land_min_sec < 0 ||
                    o.fix_to_land_max_sec < o.fix_to_land_min_sec ||
                    o.taxi_min_sec != 0) {
                    throw std::invalid_argument("invalid ARR option");
                }
            } else {
                if (!o.star.empty() || !o.fix.empty() ||
                    o.taxi_min_sec < 0 || o.entry_to_fix_min_sec != 0 ||
                    o.entry_to_fix_max_sec != 0 ||
                    o.fix_to_land_min_sec != 0 || o.fix_to_land_max_sec != 0) {
                    throw std::invalid_argument("invalid DEP option");
                }
            }
        }
    }
    for (const CrossConflict& c : p.cross_conflicts) {
        const auto a = flights.find(c.first_flight);
        const auto b = flights.find(c.second_flight);
        if (a == flights.end() || b == flights.end() ||
            a == b || c.first_before_second_sec <= 0 ||
            c.second_before_first_sec <= 0) {
            throw std::invalid_argument("invalid cross conflict");
        }
        bool a_found = false, b_found = false;
        for (const FlightOption& o : a->second->options) {
            if (o.id == c.first_option) a_found = true;
        }
        for (const FlightOption& o : b->second->options) {
            if (o.id == c.second_option) b_found = true;
        }
        if (!a_found || !b_found) {
            throw std::invalid_argument("cross conflict refers to unknown option");
        }
    }
}

bool validate_joint_solution(const JointProblem& p, const JointSolution& s,
                             std::string* reason) {
    try { validate_joint_problem(p); }
    catch (const std::exception& e) { return fail(reason, e.what()); }
    if (s.status != SolveStatus::Optimal && s.status != SolveStatus::Feasible) {
        return fail(reason, "solution status is not feasible");
    }
    if (s.assignments.size() != p.flights.size()) {
        return fail(reason, "missing or extra flight assignments");
    }
    std::map<std::string, const JointFlight*> flights;
    for (const JointFlight& f : p.flights) flights[f.id] = &f;
    std::map<std::string, const JointAssignment*> assignments;
    std::map<std::string, std::vector<TimedEvent> > runway_events, fix_events;
    std::int64_t total_delay = 0;
    for (const JointAssignment& a : s.assignments) {
        const auto f_iter = flights.find(a.flight_id);
        if (f_iter == flights.end() || !assignments.emplace(a.flight_id, &a).second) {
            return fail(reason, "unknown or repeated flight in solution");
        }
        const JointFlight& f = *f_iter->second;
        const FlightOption* chosen = nullptr;
        for (const FlightOption& o : f.options) {
            if (o.id == a.option_id) chosen = &o;
        }
        if (!chosen || a.movement != f.movement ||
            a.runway != chosen->runway || a.star != chosen->star ||
            a.fix != chosen->fix ||
            a.operation_time_sec < 0 || a.operation_time_sec > p.horizon_sec ||
            !in_windows(a.operation_time_sec, f.hard_operation_windows) ||
            !in_windows(a.operation_time_sec, chosen->operation_windows)) {
            return fail(reason, "option, operation time or time window mismatch for " + a.flight_id);
        }
        if (f.movement == Movement::Arrival) {
            const std::int64_t first = a.fix_time_sec - f.input_time_sec;
            const std::int64_t second = a.operation_time_sec - a.fix_time_sec;
            if (a.fix_time_sec < 0 || a.fix_time_sec > p.horizon_sec ||
                first < chosen->entry_to_fix_min_sec ||
                first > chosen->entry_to_fix_max_sec ||
                second < chosen->fix_to_land_min_sec ||
                second > chosen->fix_to_land_max_sec) {
                return fail(reason, "ARR route segment time violated for " + a.flight_id);
            }
            fix_events[a.fix].push_back(TimedEvent{a.flight_id, a.movement, a.fix_time_sec});
        } else {
            if (a.fix_time_sec != -1 ||
                a.operation_time_sec < f.input_time_sec + chosen->taxi_min_sec) {
                return fail(reason, "DEP ready/VTT violated for " + a.flight_id);
            }
        }
        const std::int64_t expected_delay =
            std::max<std::int64_t>(0, a.operation_time_sec - f.reference_sec);
        if (a.delay_sec != expected_delay) {
            return fail(reason, "delay inconsistent for " + a.flight_id);
        }
        total_delay += expected_delay;
        runway_events[a.runway].push_back(
            TimedEvent{a.flight_id, a.movement, a.operation_time_sec});
    }
    if (total_delay != s.total_delay_sec) {
        return fail(reason, "objective differs from recomputed total delay");
    }

    for (auto& item : runway_events) {
        std::vector<TimedEvent>& v = item.second;
        std::sort(v.begin(), v.end(), by_time);
        for (std::size_t j = 1; j < v.size(); ++j) {
            const std::int64_t required =
                p.runway_separation.between(v[j - 1].movement, v[j].movement);
            if (v[j].time - v[j - 1].time < required) {
                return fail(reason, "adjacent runway headway violation on " + item.first);
            }
        }
        // A-D-A and D-A-D must also obey nearest same-type predecessor.
        for (int type = 0; type < 2; ++type) {
            bool has_previous = false;
            std::int64_t previous = 0;
            const Movement movement = type == 0 ? Movement::Arrival : Movement::Departure;
            for (const TimedEvent& event : v) {
                if (event.movement != movement) continue;
                if (has_previous && event.time - previous <
                    p.runway_separation.between(movement, movement)) {
                    return fail(reason, "nearest same-type runway headway violation");
                }
                previous = event.time;
                has_previous = true;
            }
        }
    }
    for (auto& item : fix_events) {
        std::vector<TimedEvent>& v = item.second;
        std::sort(v.begin(), v.end(), by_time);
        for (std::size_t j = 1; j < v.size(); ++j) {
            if (v[j].time - v[j - 1].time < p.fix_gap_sec) {
                return fail(reason, "fix interval violation on " + item.first);
            }
        }
    }
    for (const CrossConflict& c : p.cross_conflicts) {
        const JointAssignment& a = *assignments.at(c.first_flight);
        const JointAssignment& b = *assignments.at(c.second_flight);
        if (a.option_id != c.first_option || b.option_id != c.second_option) continue;
        if (b.operation_time_sec - a.operation_time_sec < c.first_before_second_sec &&
            a.operation_time_sec - b.operation_time_sec < c.second_before_first_sec) {
            return fail(reason, "cross-runway conflict interval violated");
        }
    }
    if (reason) reason->clear();
    return true;
}

} // namespace runway
