#include "runway/model.h"
#include "runway/scheduler.h"

#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
void print_usage(const char* program) {
    std::cout << "Usage: " << program
              << " [--time-limit SECONDS] [--workers COUNT] [--help]\n";
}

template <typename T>
T parse_number(const std::string& text, const char* option) {
    std::istringstream stream(text);
    T result;
    char trailing;
    if (!(stream >> result) || (stream >> trailing)) {
        throw std::invalid_argument(std::string("invalid value for ") + option);
    }
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        runway::SolverOptions options;
        for (int k = 1; k < argc; ++k) {
            const std::string arg(argv[k]);
            if (arg == "--help" || arg == "-h") {
                print_usage(argv[0]);
                return 0;
            }
            if (arg == "--time-limit" && k + 1 < argc) {
                options.time_limit_seconds = parse_number<double>(argv[++k], "--time-limit");
            } else if (arg == "--workers" && k + 1 < argc) {
                options.num_workers = parse_number<int>(argv[++k], "--workers");
            } else {
                print_usage(argv[0]);
                throw std::invalid_argument("unknown or missing argument: " + arg);
            }
        }
        const runway::Problem problem = runway::make_sample_problem();
        const runway::Solution result = runway::solve_cp_sat(problem, options);
        std::cout << "CP-SAT status: " << runway::to_string(result.status) << '\n';
        if (result.status != runway::SolveStatus::Optimal &&
            result.status != runway::SolveStatus::Feasible) {
            return 1;
        }
        std::string reason;
        if (!runway::validate_solution(problem, result, &reason)) {
            std::cerr << "SOLUTION VALIDATION FAILED: " << reason << '\n';
            return 2;
        }
        std::cout << "Total delay: " << result.total_delay_sec << " sec\n";
        std::cout << "Best objective bound: " << result.objective_bound << " sec\n";
        std::cout << "All times are seconds from t=0.\n\n";
        std::cout << std::left << std::setw(5) << "Seq"
                  << std::setw(8) << "Flight"
                  << std::setw(7) << "Type"
                  << std::setw(10) << "Runway"
                  << std::right << std::setw(9) << "Time"
                  << std::setw(9) << "Delay"
                  << std::setw(9) << "Gap"
                  << std::setw(9) << "MinGap" << '\n';
        for (std::size_t k = 0; k < result.events.size(); ++k) {
            const runway::ScheduledFlight& e = result.events[k];
            std::cout << std::left << std::setw(5) << k + 1
                      << std::setw(8) << e.id
                      << std::setw(7) << runway::to_string(e.movement)
                      << std::setw(10) << e.runway
                      << std::right << std::setw(9) << e.time_sec
                      << std::setw(9) << e.delay_sec;
            if (k == 0) {
                std::cout << std::setw(9) << "-" << std::setw(9) << "-";
            } else {
                const runway::ScheduledFlight& prev = result.events[k - 1];
                std::cout << std::setw(9) << e.time_sec - prev.time_sec
                          << std::setw(9)
                          << problem.separation.between(prev.movement, e.movement);
            }
            std::cout << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 2;
    }
}
