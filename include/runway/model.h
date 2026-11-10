#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace runway {

enum class Movement { Arrival = 0, Departure = 1 };

inline const char* to_string(Movement movement) {
    return movement == Movement::Arrival ? "ARR" : "DEP";
}

struct Flight {
    std::string id;
    Movement movement;
    std::string runway;
    std::int64_t earliest_sec;  // Earliest permissible operation time.
};

// Direction matters: [leader movement][follower movement].
struct SeparationMatrix {
    std::int64_t aa;
    std::int64_t ad;
    std::int64_t da;
    std::int64_t dd;

    std::int64_t between(Movement leader, Movement follower) const {
        if (leader == Movement::Arrival) {
            return follower == Movement::Arrival ? aa : ad;
        }
        return follower == Movement::Arrival ? da : dd;
    }
};

struct Problem {
    std::vector<Flight> flights;
    SeparationMatrix separation;
};

struct ScheduledFlight {
    std::string id;
    Movement movement;
    std::string runway;
    std::int64_t time_sec;
    std::int64_t delay_sec;
};

enum class SolveStatus { Optimal, Feasible, Infeasible, Unknown, ModelInvalid };

inline const char* to_string(SolveStatus status) {
    switch (status) {
        case SolveStatus::Optimal: return "OPTIMAL";
        case SolveStatus::Feasible: return "FEASIBLE";
        case SolveStatus::Infeasible: return "INFEASIBLE";
        case SolveStatus::Unknown: return "UNKNOWN";
        case SolveStatus::ModelInvalid: return "MODEL_INVALID";
    }
    return "UNKNOWN";
}

struct Solution {
    SolveStatus status = SolveStatus::Unknown;
    std::vector<ScheduledFlight> events;  // Chronological global sequence.
    std::int64_t total_delay_sec = 0;
    double objective_bound = 0.0;
};

// Validates the sample's two exclusive runways, identifiers, and time domains.
// Throws std::invalid_argument on malformed input.
void validate_problem(const Problem& problem);

// Checks all hard constraints and delay consistency; returns false on failure.
bool validate_solution(const Problem& problem,
                       const Solution& solution,
                       std::string* reason = nullptr);

Problem make_sample_problem();

}  // namespace runway
