#include "runway/joint.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// Only this translation unit depends on C++20 OR-Tools headers.
#include "ortools/sat/cp_model.h"
#include "ortools/sat/cp_model_solver.h"
#include "ortools/sat/model.h"
#include "ortools/sat/sat_parameters.pb.h"
#include "ortools/util/sorted_interval_list.h"

namespace runway {
namespace {
namespace sat = operations_research::sat;
using operations_research::Domain;

struct FlightVars {
    sat::IntVar operation;
    sat::IntVar delay;
    std::vector<sat::BoolVar> choices;
    std::vector<sat::IntVar> fix_times;
};

struct Visit {
    std::string id;
    sat::BoolVar presence;
    sat::IntVar time;
    Movement movement;
};

Domain to_domain(const std::vector<TimeWindow>& windows,
                 std::int64_t horizon) {
    if (windows.empty()) return Domain(0, horizon);
    std::vector<operations_research::ClosedInterval> intervals;
    intervals.reserve(windows.size());
    for (const TimeWindow& w : windows) {
        intervals.push_back(operations_research::ClosedInterval(w.start_sec, w.end_sec));
    }
    return Domain::FromIntervals(intervals);
}

SolveStatus map_status(sat::CpSolverStatus status) {
    switch (status) {
        case sat::CpSolverStatus::OPTIMAL: return SolveStatus::Optimal;
        case sat::CpSolverStatus::FEASIBLE: return SolveStatus::Feasible;
        case sat::CpSolverStatus::INFEASIBLE: return SolveStatus::Infeasible;
        case sat::CpSolverStatus::MODEL_INVALID: return SolveStatus::ModelInvalid;
        default: return SolveStatus::Unknown;
    }
}

// Self-loop <=> node is absent. 0 -> first -> ... -> last -> 0 is a
// non-temporal dummy circuit which encodes a resource-local linear order.
void add_local_circuit(sat::CpModelBuilder& model,
                       const std::vector<Visit>& visits,
                       const std::string& resource_name,
                       const SeparationMatrix* runway_sep,
                       std::int64_t uniform_gap) {
    if (visits.empty()) return;
    auto circuit = model.AddCircuitConstraint();
    const auto empty = model.NewBoolVar().WithName(resource_name + "_empty");
    circuit.AddArc(0, 0, empty);
    std::vector<sat::BoolVar> any_presence;
    any_presence.reserve(visits.size() + 1);
    any_presence.push_back(empty);
    for (std::size_t i = 0; i < visits.size(); ++i) {
        const Visit& v = visits[i];
        const int node = static_cast<int>(i) + 1;
        circuit.AddArc(node, node, v.presence.Not());
        circuit.AddArc(0, node, model.NewBoolVar().WithName(resource_name + "_first_" + v.id));
        circuit.AddArc(node, 0, model.NewBoolVar().WithName(resource_name + "_last_" + v.id));
        model.AddImplication(empty, v.presence.Not());
        any_presence.push_back(v.presence);
    }
    // If the resource has no visits, dummy node 0 must choose its self-loop.
    model.AddBoolOr(any_presence);

    for (std::size_t i = 0; i < visits.size(); ++i) {
        for (std::size_t j = 0; j < visits.size(); ++j) {
            if (i == j) continue;
            const auto arc = model.NewBoolVar().WithName(
                resource_name + "_next_" + visits[i].id + "_" + visits[j].id);
            circuit.AddArc(static_cast<int>(i) + 1, static_cast<int>(j) + 1, arc);
            const std::int64_t gap = runway_sep
                ? runway_sep->between(visits[i].movement, visits[j].movement)
                : uniform_gap;
            model.AddGreaterOrEqual(visits[j].time, visits[i].time + gap)
                .OnlyEnforceIf(arc);
        }
    }
}

} // namespace

JointSolution solve_joint_cp_sat(const JointProblem& p,
                                const JointSolverOptions& options) {
    validate_joint_problem(p);
    if (options.time_limit_seconds <= 0 || options.num_workers <= 0) {
        throw std::invalid_argument("invalid time limit or worker count");
    }
    const auto build_started = std::chrono::steady_clock::now();
    sat::CpModelBuilder model;
    model.SetName("ADCO joint route runway scheduling 20");
    std::vector<FlightVars> vars(p.flights.size());
    std::map<std::string, int> flight_indices;
    std::map<std::string, std::vector<Visit> > runway_visits;
    std::map<std::string, std::vector<Visit> > fix_visits;
    // A separate same-type circuit prevents A-D-A / D-A-D from
    // bypassing the nearest ARR / DEP predecessor separation.
    std::map<std::string, std::vector<Visit> > same_type_visits;
    std::vector<sat::IntVar> delays;
    delays.reserve(p.flights.size());

    for (std::size_t i = 0; i < p.flights.size(); ++i) {
        const JointFlight& f = p.flights[i];
        FlightVars& fv = vars[i];
        flight_indices[f.id] = static_cast<int>(i);
        fv.operation = model.NewIntVar(to_domain(f.hard_operation_windows, p.horizon_sec))
            .WithName("operation_" + f.id);
        fv.delay = model.NewIntVar(Domain(0, p.horizon_sec))
            .WithName("delay_" + f.id);
        model.AddMaxEquality(fv.delay, {fv.operation - f.reference_sec, 0});
        delays.push_back(fv.delay);

        for (const FlightOption& o : f.options) {
            const auto chosen = model.NewBoolVar().WithName("choose_" + f.id + "_" + o.id);
            fv.choices.push_back(chosen);
            if (!o.operation_windows.empty()) {
                model.AddLinearConstraint(fv.operation, to_domain(o.operation_windows, p.horizon_sec))
                    .OnlyEnforceIf(chosen);
            }
            if (f.movement == Movement::Arrival) {
                auto fix_time = model.NewIntVar(Domain(0, p.horizon_sec))
                    .WithName("fix_" + f.id + "_" + o.id);
                fv.fix_times.push_back(fix_time);
                model.AddLinearConstraint(fix_time - f.input_time_sec,
                                          Domain(o.entry_to_fix_min_sec, o.entry_to_fix_max_sec))
                    .OnlyEnforceIf(chosen);
                model.AddLinearConstraint(fv.operation - fix_time,
                                          Domain(o.fix_to_land_min_sec, o.fix_to_land_max_sec))
                    .OnlyEnforceIf(chosen);
                fix_visits[o.fix].push_back(Visit{f.id + "_" + o.id, chosen, fix_time,
                                                   f.movement});
            } else {
                // Unused dummy variable for a uniform [flight][option] layout.
                fv.fix_times.push_back(model.NewConstant(-1));
                model.AddGreaterOrEqual(fv.operation, f.input_time_sec + o.taxi_min_sec)
                    .OnlyEnforceIf(chosen);
            }
        }
        model.AddExactlyOne(fv.choices);

        // A flight is represented by one runway node, even if two options
        // of that flight share a runway. Presence = sum(selected options using r).
        for (const std::string& runway : p.runways) {
            std::vector<sat::BoolVar> matching;
            for (std::size_t k = 0; k < f.options.size(); ++k) {
                if (f.options[k].runway == runway) matching.push_back(fv.choices[k]);
            }
            if (matching.empty()) continue;
            const auto present = model.NewBoolVar().WithName("use_" + f.id + "_" + runway);
            model.AddEquality(present, sat::LinearExpr::Sum(matching));
            const Visit v{f.id, present, fv.operation, f.movement};
            runway_visits[runway].push_back(v);
            same_type_visits[runway + (f.movement == Movement::Arrival ? "_ARR" : "_DEP")]
                .push_back(v);
        }
    }

    for (const std::string& runway : p.runways) {
        add_local_circuit(model, runway_visits[runway], "runway_" + runway,
                          &p.runway_separation, 0);
        add_local_circuit(model, same_type_visits[runway + "_ARR"],
                          "same_arr_" + runway, &p.runway_separation, 0);
        add_local_circuit(model, same_type_visits[runway + "_DEP"],
                          "same_dep_" + runway, &p.runway_separation, 0);
    }
    for (const std::string& fix : p.fixes) {
        add_local_circuit(model, fix_visits[fix], "fix_" + fix,
                          nullptr, p.fix_gap_sec);
    }

    for (const CrossConflict& c : p.cross_conflicts) {
        const int ai = flight_indices.at(c.first_flight);
        const int bi = flight_indices.at(c.second_flight);
        const JointFlight& a = p.flights[ai];
        const JointFlight& b = p.flights[bi];
        std::size_t ak = 0, bk = 0;
        while (a.options[ak].id != c.first_option) ++ak;
        while (b.options[bk].id != c.second_option) ++bk;
        const auto before = model.NewBoolVar().WithName(
            "cross_before_" + c.first_flight + "_" + c.second_flight);
        const std::vector<sat::BoolVar> forward_guards = {
            vars[ai].choices[ak], vars[bi].choices[bk], before};
        const std::vector<sat::BoolVar> reverse_guards = {
            vars[ai].choices[ak], vars[bi].choices[bk], before.Not()};
        model.AddGreaterOrEqual(vars[bi].operation,
                                vars[ai].operation + c.first_before_second_sec)
            .OnlyEnforceIf(forward_guards);
        model.AddGreaterOrEqual(vars[ai].operation,
                                vars[bi].operation + c.second_before_first_sec)
            .OnlyEnforceIf(reverse_guards);
    }

    model.Minimize(sat::LinearExpr::Sum(delays));
    const auto build_finished = std::chrono::steady_clock::now();
    sat::SatParameters parameters;
    parameters.set_max_time_in_seconds(options.time_limit_seconds);
    parameters.set_num_search_workers(options.num_workers);
    parameters.set_log_search_progress(options.log_search);
    sat::Model solver_context;
    solver_context.Add(sat::NewSatParameters(parameters));
    const auto solve_started = std::chrono::steady_clock::now();
    const sat::CpSolverResponse response = sat::SolveCpModel(model.Build(), &solver_context);
    const auto solve_finished = std::chrono::steady_clock::now();

    JointSolution result;
    result.status = map_status(response.status());
    result.best_objective_bound = response.best_objective_bound();
    result.build_time_ms = std::chrono::duration<double, std::milli>(
        build_finished - build_started).count();
    result.solve_time_ms = std::chrono::duration<double, std::milli>(
        solve_finished - solve_started).count();
    result.conflicts = response.num_conflicts();
    result.branches = response.num_branches();
    if (result.status != SolveStatus::Optimal && result.status != SolveStatus::Feasible) {
        return result;
    }
    for (std::size_t i = 0; i < p.flights.size(); ++i) {
        const JointFlight& f = p.flights[i];
        const FlightVars& fv = vars[i];
        std::size_t chosen_k = f.options.size();
        for (std::size_t k = 0; k < f.options.size(); ++k) {
            if (sat::SolutionBooleanValue(response, fv.choices[k])) { chosen_k = k; break; }
        }
        if (chosen_k == f.options.size()) {
            throw std::runtime_error("solver returned no chosen option for " + f.id);
        }
        const FlightOption& o = f.options[chosen_k];
        const std::int64_t operation = sat::SolutionIntegerValue(response, fv.operation);
        const std::int64_t delay = sat::SolutionIntegerValue(response, fv.delay);
        const std::int64_t fix_time = f.movement == Movement::Arrival
            ? sat::SolutionIntegerValue(response, fv.fix_times[chosen_k]) : -1;
        result.assignments.push_back(JointAssignment{
            f.id, f.movement, o.id, o.runway, o.star, o.fix,
            fix_time, operation, delay});
        result.total_delay_sec += delay;
    }
    std::string reason;
    if (!validate_joint_solution(p, result, &reason)) {
        throw std::runtime_error("independent joint validator rejected solution: " + reason);
    }
    return result;
}

} // namespace runway
