#include "runway/joint.h"

#include <algorithm>
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
template <typename T>
T parse(const std::string& text, const char* name) {
    std::istringstream s(text);
    T value;
    char tail;
    if (!(s >> value) || (s >> tail)) {
        throw std::invalid_argument(std::string("invalid value for ") + name);
    }
    return value;
}
void usage(const char* program) {
    std::cout << "Usage: " << program << " [--scenario baseline20|route-switch|fix-conflict|infeasible]"
              << " [--time-limit SEC] [--workers N] [--log] [--help]\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        std::string scenario = "baseline20";
        runway::JointSolverOptions options;
        for (int i = 1; i < argc; ++i) {
            const std::string arg(argv[i]);
            if (arg == "--help" || arg == "-h") { usage(argv[0]); return 0; }
            if (arg == "--scenario" && i + 1 < argc) scenario = argv[++i];
            else if (arg == "--time-limit" && i + 1 < argc)
                options.time_limit_seconds = parse<double>(argv[++i], "--time-limit");
            else if (arg == "--workers" && i + 1 < argc)
                options.num_workers = parse<int>(argv[++i], "--workers");
            else if (arg == "--log") options.log_search = true;
            else { usage(argv[0]); throw std::invalid_argument("unknown or missing argument: " + arg); }
        }
        const runway::JointProblem p = runway::make_joint_problem(scenario);
        std::size_t count_options = 0;
        for (const auto& f : p.flights) count_options += f.options.size();
        std::cout << "Scenario: " << scenario << ", flights: " << p.flights.size()
                  << ", options: " << count_options << ", runways: " << p.runways.size()
                  << ", shared fixes: " << p.fixes.size() << '\n';
        const runway::JointSolution solution = runway::solve_joint_cp_sat(p, options);
        std::cout << "CP-SAT status: " << runway::to_string(solution.status) << '\n'
                  << "Build time: " << std::fixed << std::setprecision(1)
                  << solution.build_time_ms << " ms, solve time: " << solution.solve_time_ms
                  << " ms, branches: " << solution.branches
                  << ", conflicts: " << solution.conflicts << '\n';
        if (solution.status != runway::SolveStatus::Optimal &&
            solution.status != runway::SolveStatus::Feasible) {
            std::cerr << "No publishable schedule; status is not FEASIBLE/OPTIMAL.\n";
            return solution.status == runway::SolveStatus::Infeasible ? 3 : 1;
        }
        std::string reason;
        if (!runway::validate_joint_solution(p, solution, &reason)) {
            std::cerr << "VALIDATION FAILED: " << reason << '\n';
            return 2;
        }
        std::cout << "Total delay: " << solution.total_delay_sec
                  << " sec; best objective bound: " << solution.best_objective_bound
                  << " sec; independently validated: YES\n";
        std::vector<runway::JointAssignment> events = solution.assignments;
        std::sort(events.begin(), events.end(),
            [](const runway::JointAssignment& a, const runway::JointAssignment& b) {
                if (a.operation_time_sec != b.operation_time_sec)
                    return a.operation_time_sec < b.operation_time_sec;
                return a.flight_id < b.flight_id;
            });
        std::cout << std::left << std::setw(8) << "Flight" << std::setw(6) << "Type"
                  << std::setw(7) << "Option" << std::setw(7) << "Runway"
                  << std::setw(10) << "STAR" << std::setw(7) << "Fix"
                  << std::right << std::setw(9) << "FixTime"
                  << std::setw(10) << "CLDT/MTOT" << std::setw(8) << "Delay" << '\n';
        for (const auto& a : events) {
            std::cout << std::left << std::setw(8) << a.flight_id
                      << std::setw(6) << runway::to_string(a.movement)
                      << std::setw(7) << a.option_id
                      << std::setw(7) << a.runway
                      << std::setw(10) << (a.star.empty() ? "-" : a.star)
                      << std::setw(7) << (a.fix.empty() ? "-" : a.fix)
                      << std::right << std::setw(9)
                      << (a.fix_time_sec < 0 ? "-" : std::to_string(a.fix_time_sec))
                      << std::setw(10) << a.operation_time_sec
                      << std::setw(8) << a.delay_sec << '\n';
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 2;
    }
}
