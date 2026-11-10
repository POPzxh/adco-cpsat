#pragma once

#include "runway/model.h"

#include <cstdint>
#include <string>
#include <vector>

namespace runway {

// All intervals are inclusive and all times are integer seconds from t=0.
struct TimeWindow {
    std::int64_t start_sec = 0;
    std::int64_t end_sec = 0;
    TimeWindow() = default;
    TimeWindow(std::int64_t start, std::int64_t end)
        : start_sec(start), end_sec(end) {}
};

// An ARR option represents runway + STAR + route through one shared fix.
// A DEP option represents runway + minimum ready-to-takeoff (VTT) duration.
struct FlightOption {
    std::string id;
    std::string runway;
    std::string star;        // ARR only
    std::string fix;         // ARR only, one visit per option in V0.3
    std::int64_t entry_to_fix_min_sec = 0;
    std::int64_t entry_to_fix_max_sec = 0;
    std::int64_t fix_to_land_min_sec = 0;
    std::int64_t fix_to_land_max_sec = 0;
    std::int64_t taxi_min_sec = 0;  // DEP only
    std::vector<TimeWindow> operation_windows; // Active iff this option is selected.
};

struct JointFlight {
    std::string id;
    Movement movement = Movement::Arrival;
    std::int64_t input_time_sec = 0; // Fixed ENTRY for ARR, READY for DEP.
    std::int64_t reference_sec = 0;  // Fixed business baseline for delay.
    std::vector<TimeWindow> hard_operation_windows; // e.g. NTOT; applies to all options.
    std::vector<FlightOption> options;
};

// Only active if BOTH specified options are selected. Time reference is
// operation time (CLDT/MTOT) in this prototype, not any route fix time.
struct CrossConflict {
    std::string first_flight;
    std::string first_option;
    std::string second_flight;
    std::string second_option;
    std::int64_t first_before_second_sec = 0;
    std::int64_t second_before_first_sec = 0;
    CrossConflict() = default;
    CrossConflict(const std::string& af, const std::string& ao,
                  const std::string& bf, const std::string& bo,
                  std::int64_t ab, std::int64_t ba)
        : first_flight(af), first_option(ao), second_flight(bf),
          second_option(bo), first_before_second_sec(ab),
          second_before_first_sec(ba) {}
};

struct JointProblem {
    std::int64_t horizon_sec = 5400;
    std::vector<std::string> runways;
    std::vector<std::string> fixes;
    SeparationMatrix runway_separation{120, 120, 90, 100};
    std::int64_t fix_gap_sec = 90;
    std::vector<JointFlight> flights;
    std::vector<CrossConflict> cross_conflicts;
};

struct JointAssignment {
    std::string flight_id;
    Movement movement = Movement::Arrival;
    std::string option_id;
    std::string runway;
    std::string star;
    std::string fix;
    std::int64_t fix_time_sec = -1;  // -1 for DEP.
    std::int64_t operation_time_sec = 0;
    std::int64_t delay_sec = 0;
    JointAssignment() = default;
    JointAssignment(const std::string& f, Movement m, const std::string& option,
                    const std::string& r, const std::string& s,
                    const std::string& x, std::int64_t ft,
                    std::int64_t operation, std::int64_t delay)
        : flight_id(f), movement(m), option_id(option), runway(r), star(s),
          fix(x), fix_time_sec(ft), operation_time_sec(operation),
          delay_sec(delay) {}
};

struct JointSolution {
    SolveStatus status = SolveStatus::Unknown;
    std::vector<JointAssignment> assignments;
    std::int64_t total_delay_sec = 0;
    double best_objective_bound = 0.0;
    double build_time_ms = 0.0;
    double solve_time_ms = 0.0;
    std::int64_t conflicts = 0;
    std::int64_t branches = 0;
};

struct JointSolverOptions {
    double time_limit_seconds = 3.0;
    int num_workers = 4;
    bool log_search = false;
};

// Scenarios: baseline20 (default), route-switch, fix-conflict, infeasible.
JointProblem make_joint_problem(const std::string& scenario = "baseline20");

// Public C++11 API, independent of OR-Tools.
void validate_joint_problem(const JointProblem& problem);
bool validate_joint_solution(const JointProblem& problem,
                             const JointSolution& solution,
                             std::string* reason = nullptr);

// Implemented in a separate C++20 translation unit linked with OR-Tools 9.14.
JointSolution solve_joint_cp_sat(
    const JointProblem& problem,
    const JointSolverOptions& options = JointSolverOptions());

} // namespace runway
