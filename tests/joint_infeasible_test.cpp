#include "runway/joint.h"

#include <iostream>

int main() {
    const runway::JointProblem p = runway::make_joint_problem("infeasible");
    runway::JointSolverOptions options;
    options.time_limit_seconds = 5.0;
    options.num_workers = 2;
    const runway::JointSolution s = runway::solve_joint_cp_sat(p, options);
    if (s.status != runway::SolveStatus::Infeasible) {
        std::cerr << "Expected a proof of infeasibility, got "
                  << runway::to_string(s.status) << '\n';
        return 1;
    }
    if (!s.assignments.empty()) return 2;
    return 0;
}
