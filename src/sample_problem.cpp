#include "runway/model.h"

namespace runway {

Problem make_sample_problem() {
    Problem problem;
    // Example-only seconds, not operationally approved separation standards.
    problem.separation = SeparationMatrix{120, 90, 150, 100};
    problem.flights = {
        {"A01", Movement::Arrival,   "RWY-ARR",   0},
        {"D01", Movement::Departure, "RWY-DEP",  20},
        {"A02", Movement::Arrival,   "RWY-ARR",  80},
        {"D02", Movement::Departure, "RWY-DEP", 105},
        {"A03", Movement::Arrival,   "RWY-ARR", 155},
        {"D03", Movement::Departure, "RWY-DEP", 190},
        {"A04", Movement::Arrival,   "RWY-ARR", 255},
        {"D04", Movement::Departure, "RWY-DEP", 275},
        {"A05", Movement::Arrival,   "RWY-ARR", 340},
        {"D05", Movement::Departure, "RWY-DEP", 365},
    };
    return problem;
}

}  // namespace runway
