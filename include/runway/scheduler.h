#pragma once

#include "runway/model.h"

namespace runway {

struct SolverOptions {
    double time_limit_seconds = 10.0;
    int num_workers = 4;
};

// The solver's C++20 implementation is behind a C++11-compatible interface.
// Requires linking against the runway_cp_sat target.
Solution solve_cp_sat(const Problem& problem,
                      const SolverOptions& options = SolverOptions());

}  // namespace runway
