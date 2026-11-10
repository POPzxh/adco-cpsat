#include "runway/joint.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
runway::FlightOption arr() {
    runway::FlightOption o;
    o.id = "P1"; o.runway = "R1"; o.star = "S"; o.fix = "X";
    o.entry_to_fix_min_sec = 100; o.entry_to_fix_max_sec = 300;
    o.fix_to_land_min_sec = 150; o.fix_to_land_max_sec = 500;
    return o;
}
runway::FlightOption dep() {
    runway::FlightOption o;
    o.id = "Q1"; o.runway = "R1"; o.taxi_min_sec = 0;
    return o;
}
runway::JointSolution solution(std::int64_t last_time) {
    runway::JointSolution s;
    s.status = runway::SolveStatus::Feasible;
    s.assignments = {
        {"A1", runway::Movement::Arrival, "P1", "R1", "S", "X", 120, 300, 300},
        {"D1", runway::Movement::Departure, "Q1", "R1", "", "", -1, 420, 420},
        {"A2", runway::Movement::Arrival, "P1", "R1", "S", "X", 250, last_time, last_time},
    };
    s.total_delay_sec = 300 + 420 + last_time;
    return s;
}
} // namespace

int main() {
    const runway::JointProblem demo = runway::make_joint_problem();
    runway::validate_joint_problem(demo);
    if (demo.flights.size() != 20) return 1;
    std::size_t options = 0;
    for (const auto& f : demo.flights) options += f.options.size();
    if (options != 48) return 2;
    if (demo.flights.front().options.front().operation_windows.empty()) return 3;
    // A completely independent feasible witness for the 20-flight baseline.
    // This is NOT hard-coded into the solver or required as its initial plan.
    runway::JointSolution witness;
    witness.status = runway::SolveStatus::Feasible;
    for (const auto& f : demo.flights) {
        const bool arrival = f.movement == runway::Movement::Arrival;
        const std::int64_t time = f.input_time_sec + (arrival ? 390 : 180);
        const std::int64_t delay = std::max<std::int64_t>(0, time - f.reference_sec);
        witness.assignments.push_back(runway::JointAssignment{
            f.id, f.movement, arrival ? "P2" : "Q1", arrival ? "R2" : "R1",
            arrival ? "STAR-S" : "", arrival ? "Y" : "",
            arrival ? f.input_time_sec + 180 : -1, time, delay});
        witness.total_delay_sec += delay;
    }
    std::string witness_error;
    if (!runway::validate_joint_solution(demo, witness, &witness_error)) {
        std::cerr << "20-flight constructive witness invalid: " << witness_error << '\n';
        return 6;
    }
    // Make A01 use P1 without rearranging D01: the shared runway must reject it.
    runway::JointSolution rejected = witness;
    auto& a01 = rejected.assignments.front();
    a01.option_id = "P1"; a01.runway = "R1"; a01.star = "STAR-N";
    a01.fix = "X"; a01.fix_time_sec = 420; a01.operation_time_sec = 600;
    rejected.total_delay_sec -= a01.delay_sec;
    a01.delay_sec = 0;
    if (runway::validate_joint_solution(demo, rejected, &witness_error)) {
        std::cerr << "same-runway A01/P1 vs D01 NTOT collision not caught\n";
        return 7;
    }

    runway::JointProblem p;
    p.runways = {"R1"}; p.fixes = {"X"}; p.horizon_sec = 1000;
    p.runway_separation = runway::SeparationMatrix{300, 120, 90, 100};
    p.fix_gap_sec = 90;
    runway::JointFlight a1;
    a1.id = "A1"; a1.movement = runway::Movement::Arrival;
    a1.options.push_back(arr());
    runway::JointFlight d1;
    d1.id = "D1"; d1.movement = runway::Movement::Departure;
    d1.options.push_back(dep());
    runway::JointFlight a2 = a1; a2.id = "A2";
    p.flights = {a1, d1, a2};
    std::string error;
    if (!runway::validate_joint_solution(p, solution(600), &error)) {
        std::cerr << "valid plan rejected: " << error << '\n';
        return 4;
    }
    if (runway::validate_joint_solution(p, solution(510), &error)) {
        std::cerr << "A-D-A accidentally bypassed nearest A-A protection\n";
        return 5;
    }
    const auto bad = runway::make_joint_problem("infeasible");
    runway::validate_joint_problem(bad); // Model is well-formed, but no valid schedule.
    return 0;
}
