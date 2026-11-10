#include "runway/scheduler.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// Deliberately confined to this translation unit: the public API is C++11.
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_parameters.pb.h"

namespace runway {
namespace {
namespace sat = operations_research::sat;

std::int64_t compute_horizon(const Problem& problem) {
    std::int64_t last_release = 0;
    for (const Flight& f : problem.flights) {
        last_release = std::max(last_release, f.earliest_sec);
    }
    const SeparationMatrix& s = problem.separation;
    const std::int64_t largest_gap =
        std::max(std::max(s.aa, s.ad), std::max(s.da, s.dd));
    const std::int64_t count = static_cast<std::int64_t>(problem.flights.size());
    // At time max(earliest) a trivial serial sequence is always feasible.
    // Bounds are deliberately capped for a small educational demonstration.
    if (count > 10000 || last_release > 1000000000LL ||
        largest_gap > 1000000000LL) {
        throw std::invalid_argument("demo horizon limits exceeded");
    }
    return last_release + count * largest_gap;
}

SolveStatus to_status(sat::CpSolverStatus s) {
    switch (s) {
        case sat::CpSolverStatus::OPTIMAL: return SolveStatus::Optimal;
        case sat::CpSolverStatus::FEASIBLE: return SolveStatus::Feasible;
        case sat::CpSolverStatus::INFEASIBLE: return SolveStatus::Infeasible;
        case sat::CpSolverStatus::MODEL_INVALID: return SolveStatus::ModelInvalid;
        default: return SolveStatus::Unknown;
    }
}
}  // namespace

Solution solve_cp_sat(const Problem& problem, const SolverOptions& options) {
    validate_problem(problem);
    if (options.time_limit_seconds <= 0.0 || options.num_workers <= 0) {
        throw std::invalid_argument("time limit and worker count must be positive");
    }

    const int n = static_cast<int>(problem.flights.size());
    const std::int64_t horizon = compute_horizon(problem);
    sat::CpModelBuilder model;
    std::vector<sat::IntVar> times;
    std::vector<sat::IntVar> delays;
    times.reserve(n);
    delays.reserve(n);

    for (const Flight& flight : problem.flights) {
        // Every operation is mandatory and cannot precede earliest_sec.
        const sat::IntVar t = model.NewIntVar(
            operations_research::Domain(flight.earliest_sec, horizon))
            .WithName("t_" + flight.id);
        const sat::IntVar d = model.NewIntVar(
            operations_research::Domain(0, horizon - flight.earliest_sec))
            .WithName("delay_" + flight.id);
        model.AddEquality(d, t - flight.earliest_sec);
        times.push_back(t);
        delays.push_back(d);
    }

    // 0 is a dummy node. A single Hamiltonian circuit through 0 and all
    // flights encodes a chronological total order without placing any flight
    // on an optional self-loop.
    auto circuit = model.AddCircuitConstraint();
    for (int i = 0; i < n; ++i) {
        const Flight& fi = problem.flights[i];
        circuit.AddArc(0, i + 1,
            model.NewBoolVar().WithName("first_" + fi.id));
        circuit.AddArc(i + 1, 0,
            model.NewBoolVar().WithName("last_" + fi.id));

        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            const Flight& fj = problem.flights[j];
            const sat::BoolVar next = model.NewBoolVar().WithName(
                "next_" + fi.id + "_" + fj.id);
            circuit.AddArc(i + 1, j + 1, next);

            // AA/AD/DA/DD apply to consecutive *global sequence* events.
            const std::int64_t min_gap =
                problem.separation.between(fi.movement, fj.movement);
            model.AddGreaterOrEqual(times[j], times[i] + min_gap)
                .OnlyEnforceIf(next);
        }
    }

    // Crucial additional physical-runway rule:
    // A-D-A interleaving does NOT waive the arrival-runway AA separation.
    // Likewise, D-A-D does NOT waive the departure-runway DD separation.
    // For each pair on the same runway, choose which event occurs first.
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const Flight& fi = problem.flights[i];
            const Flight& fj = problem.flights[j];
            if (fi.runway != fj.runway) continue;
            const std::int64_t min_gap =
                problem.separation.between(fi.movement, fj.movement);
            const sat::BoolVar i_before_j = model.NewBoolVar().WithName(
                "runway_before_" + fi.id + "_" + fj.id);
            model.AddGreaterOrEqual(times[j], times[i] + min_gap)
                .OnlyEnforceIf(i_before_j);
            model.AddGreaterOrEqual(times[i], times[j] + min_gap)
                .OnlyEnforceIf(i_before_j.Not());
        }
    }

    model.Minimize(sat::LinearExpr::Sum(delays));
    sat::SatParameters parameters;
    parameters.set_max_time_in_seconds(options.time_limit_seconds);
    parameters.set_num_search_workers(options.num_workers);
    sat::Model solver_context;
    solver_context.Add(sat::NewSatParameters(parameters));

    const sat::CpSolverResponse response =
        sat::SolveCpModel(model.Build(), &solver_context);
    Solution result;
    result.status = to_status(response.status());
    result.objective_bound = response.best_objective_bound();
    if (result.status != SolveStatus::Optimal &&
        result.status != SolveStatus::Feasible) {
        return result;
    }

    for (int i = 0; i < n; ++i) {
        const Flight& f = problem.flights[i];
        const std::int64_t t = sat::SolutionIntegerValue(response, times[i]);
        const std::int64_t d = sat::SolutionIntegerValue(response, delays[i]);
        result.events.push_back(ScheduledFlight{f.id, f.movement, f.runway, t, d});
        result.total_delay_sec += d;
    }
    std::sort(result.events.begin(), result.events.end(),
              [](const ScheduledFlight& a, const ScheduledFlight& b) {
                  if (a.time_sec != b.time_sec) return a.time_sec < b.time_sec;
                  return a.id < b.id;
              });
    return result;
}

}  // namespace runway
