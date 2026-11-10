#include "runway/joint.h"

#include <iostream>
#include <string>

int main() {
    const runway::JointProblem p = runway::make_joint_problem();
    runway::JointSolverOptions opts;
    opts.time_limit_seconds = 20.0;
    opts.num_workers = 4;
    const runway::JointSolution s = runway::solve_joint_cp_sat(p, opts);
    if (s.status != runway::SolveStatus::Optimal &&
        s.status != runway::SolveStatus::Feasible) {
        std::cerr << "No feasible solution, status: " << runway::to_string(s.status) << '\n';
        return 1;
    }
    std::string reason;
    if (!runway::validate_joint_solution(p, s, &reason)) {
        std::cerr << "Invalid solution: " << reason << '\n';
        return 2;
    }
    bool seen_a01 = false, seen_d01 = false;
    for (const auto& a : s.assignments) {
        if (a.flight_id == "A01") {
            if (a.option_id != "P2") {
                std::cerr << "A01 should be forced to P2 by D01 NTOT\n";
                return 3;
            }
            seen_a01 = true;
        }
        if (a.flight_id == "D01") {
            if (a.runway != "R1" || a.operation_time_sec > 690) return 4;
            seen_d01 = true;
        }
    }
    return (seen_a01 && seen_d01) ? 0 : 5;
}
