#include "runway/model.h"
#include "runway/scheduler.h"

#include <iostream>
#include <string>

int main() {
    const runway::Problem p = runway::make_sample_problem();
    runway::SolverOptions options;
    options.time_limit_seconds = 30.0;
    options.num_workers = 4;
    const runway::Solution result = runway::solve_cp_sat(p, options);
    if (result.status != runway::SolveStatus::Optimal &&
        result.status != runway::SolveStatus::Feasible) {
        std::cerr << "No feasible CP-SAT solution; status="
                  << runway::to_string(result.status) << '\n';
        return 1;
    }
    std::string reason;
    if (!runway::validate_solution(p, result, &reason)) {
        std::cerr << "Invalid solution: " << reason << '\n';
        return 1;
    }
    if (static_cast<double>(result.total_delay_sec) + 1e-5 < result.objective_bound) {
        std::cerr << "Best bound exceeds feasible objective\n";
        return 1;
    }
    // Independently verified for this fixed 10-flight example.
    if (result.status == runway::SolveStatus::Optimal &&
        result.total_delay_sec != 2945) {
        std::cerr << "Unexpected optimum: " << result.total_delay_sec << '\n';
        return 1;
    }
    return 0;
}
