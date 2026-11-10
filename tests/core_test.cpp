#include "runway/model.h"

#include <cstdint>
#include <iostream>
#include <string>

namespace {
bool require(bool condition, const char* label) {
    if (!condition) std::cerr << "FAILED: " << label << '\n';
    return condition;
}

runway::Solution make_serial_solution(const runway::Problem& problem) {
    runway::Solution result;
    result.status = runway::SolveStatus::Feasible;
    std::int64_t previous_time = -100000;
    runway::Movement previous_type = runway::Movement::Arrival;
    for (const runway::Flight& f : problem.flights) {
        std::int64_t t = f.earliest_sec;
        if (!result.events.empty()) {
            const std::int64_t gap = problem.separation.between(previous_type, f.movement);
            if (t < previous_time + gap) t = previous_time + gap;
        }
        const std::int64_t delay = t - f.earliest_sec;
        result.events.push_back(runway::ScheduledFlight{f.id, f.movement, f.runway, t, delay});
        result.total_delay_sec += delay;
        previous_time = t;
        previous_type = f.movement;
    }
    return result;
}
}  // namespace

int main() {
    const runway::Problem p = runway::make_sample_problem();
    runway::validate_problem(p);
    if (!require(p.flights.size() == 10, "10 sample flights")) return 1;
    if (!require(p.separation.between(runway::Movement::Arrival, runway::Movement::Arrival) == 120, "AA")) return 1;
    if (!require(p.separation.between(runway::Movement::Arrival, runway::Movement::Departure) == 90, "AD")) return 1;
    if (!require(p.separation.between(runway::Movement::Departure, runway::Movement::Arrival) == 150, "DA")) return 1;
    if (!require(p.separation.between(runway::Movement::Departure, runway::Movement::Departure) == 100, "DD")) return 1;

    const runway::Solution baseline = make_serial_solution(p);
    std::string reason;
    if (!require(runway::validate_solution(p, baseline, &reason), "serial feasible schedule")) {
        std::cerr << reason << '\n';
        return 1;
    }
    runway::Solution bad = baseline;
    bad.events[1].time_sec = bad.events[0].time_sec + 1;
    bad.events[1].delay_sec = bad.events[1].time_sec - p.flights[1].earliest_sec;
    bad.total_delay_sec = 0;
    for (const auto& e : bad.events) bad.total_delay_sec += e.delay_sec;
    if (!require(!runway::validate_solution(p, bad, &reason), "reject global gap violation")) return 1;

    bad = baseline;
    bad.events[0].runway = "RWY-DEP";
    if (!require(!runway::validate_solution(p, bad, &reason), "reject wrong runway")) return 1;

    // A(0) -> D(10) -> A(20) satisfies AD=10 and DA=10, but violates
    // AA=200 for the *same physical arrival runway*.
    runway::Problem interleaved;
    interleaved.separation = runway::SeparationMatrix{200, 10, 10, 100};
    interleaved.flights = {
        {"A01", runway::Movement::Arrival, "RWY-ARR", 0},
        {"D01", runway::Movement::Departure, "RWY-DEP", 10},
        {"A02", runway::Movement::Arrival, "RWY-ARR", 20},
    };
    runway::Solution unsafe;
    unsafe.status = runway::SolveStatus::Feasible;
    unsafe.events = {
        {"A01", runway::Movement::Arrival, "RWY-ARR", 0, 0},
        {"D01", runway::Movement::Departure, "RWY-DEP", 10, 0},
        {"A02", runway::Movement::Arrival, "RWY-ARR", 20, 0},
    };
    if (!require(!runway::validate_solution(interleaved, unsafe, &reason),
                 "reject same-runway gap hidden by interleaving")) return 1;
    return 0;
}
