# Runway CP-SAT V0.3 — 20-flight joint route/runway scheduling

This is an **in-place evolution of `runway-cpsat-v2`**, not a new unrelated
repository. The former 10-flight API, original demo, and regression tests are
retained. The default `runway_demo` now demonstrates a **joint 20-flight model**.

> **Safety:** every time, runway spacing and fix spacing used here is synthetic.
> This is an algorithms prototype, **not** an operational air-traffic system.

## New model

- **20 mandatory flights:** A01–A10 ARR and D01–D10 DEP.
- **48 candidate options:** 29 ARR (P1/P2/P3) and 19 DEP (Q1/Q2).
- **2 mixed-use runways:** R1, R2. An ARR option chooses a runway + STAR +
  route fix; a DEP option chooses a runway + runway-specific VTT.
- **ARR time chain:** fixed ENTRY → chosen FIX → CLDT, with two conditional
  interval-duration constraints. One `CLDT` variable per flight.
- **DEP time chain:** fixed READY + option-specific min taxi time ≤ MTOT.
- **Hard time windows:** per-flight (NTOT) and optional per-option windows.
  Windows can be a union of disjoint closed intervals.
- **Runway-local circuit:** an absent flight uses its self-loop; selected
  neighbors receive AA/AD/DA/DD directed minimum gaps.
- **Nearest same-type predecessor:** additional ARR-only and DEP-only
  circuits on each runway. This prevents an A-D-A sequence from skipping AA.
- **Shared fix circuits:** X and Y use **option-specific visit nodes**, not
  the flight's runway node. Only chosen routes occupy the relevant fix.
- **Conditional cross-runway conflict:** A03/P3 vs D03/Q1, with asymmetric
  directed headways. Only applies if **both** named options are chosen.
- **Objective:** minimize sum of `max(0, operation_time - reference_time)`.
- **Independent validator:** recomputes segment times, choice, windows,
  runway neighbors, same-type neighbors, fix neighbors, cross-runway conflicts
  and objective *without reading the model's Circuit literals*.

This v0.3 models one shared fix visit per arrival route option. **Not yet
implemented:** arbitrary route graphs with multiple visits to the same fix,
ADA/DAD triple rules, runway closures, real holding-capacity semantics, live
input snapshot loading, rolling-horizon publication. These will require
confirmed business rules, and are **not** silently approximated.

## 1. Build with locally installed OR-Tools 9.14

Requirements: Linux, CMake >= 3.24, Ninja (optional), a compiler with C++20,
locally installed OR-Tools 9.14 and its installed transitive dependencies.
As on your machine: GCC/G++ 13 and `/usr/local/ortools-9.14`.

```bash
cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/usr/local/ortools-9.14 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-13
cmake --build build -j4
ctest --test-dir build --output-on-failure
```

You can also use a preset:

```bash
cmake --preset ninja-ortools
cmake --build --preset ninja-ortools
ctest --preset ninja-ortools --output-on-failure
```

If VS Code already provides the compiler and `CMAKE_PREFIX_PATH` in the
configure command, no new `.vscode/settings.json` is required.

## 2. Run

```bash
# New default: 20-flight ARR/DEP joint routing/runway optimization
./build/runway_demo --time-limit 3 --workers 4

# More thorough search, including native CP-SAT progress logs
./build/runway_demo --scenario baseline20 --time-limit 10 --workers 4 --log

# Relax the A01/P1 special landing window
./build/runway_demo --scenario route-switch --time-limit 3

# Increase FIX X/Y headway from 90 to 180 seconds
./build/runway_demo --scenario fix-conflict --time-limit 3

# Intentionally infeasible D01 hard window; expect INFEASIBLE (exit 3)
./build/runway_demo --scenario infeasible --time-limit 3

# Original 10-flight global-sequence demonstration is unchanged
./build/runway_legacy_demo --time-limit 1 --workers 4
```

The demo shows the chosen option, runway, STAR, fix, fix time, CLDT/MTOT,
delay, bound, solver status and timing. `FEASIBLE` is not an optimality proof.
The application refuses to publish `UNKNOWN`, `INFEASIBLE` or `MODEL_INVALID`.

### Important benchmark scenario

- A01/P1: R1, CLDT `[600,650]`.
- D01/Q1: R1, MTOT `[630,690]`, earliest 630.
- If A01 is first: D01 ≥ A01 + AD(120) ≥ 720 > 690.
- If D01 is first: A01 ≥ D01 + DA(90) ≥ 720 > 650.
- Thus **A01/P1 is impossible**, even though this route has a shorter
  nominal total duration than P2. The model must choose A01/P2 on R2.
- An independently feasible illustrative *choice pattern* is all ARR on P2,
  all DEP on Q1 (the solver chooses and schedules times itself).

### Hard separation table (seconds, synthetic)

| Leader \\ Follower | ARR | DEP |
|---|---:|---:|
| ARR | 120 (AA) | 120 (AD) |
| DEP | 90 (DA) | 100 (DD) |

For both X and Y fixes the baseline headway is 90 seconds.
Runways are independently usable except for explicit cross-conflict rules.

## 3. Core-only build for unit tests (no OR-Tools needed)

```bash
cmake -S . -B build-core -G Ninja -DRUNWAY_WITH_ORTOOLS=OFF
cmake --build build-core
ctest --test-dir build-core --output-on-failure
```

The public domain, scenario and validator API remains **C++11**. Only
`src/cp_sat_scheduler.cpp` and `src/joint/cp_sat_joint.cpp` require C++20.

## 4. Project layout

```text
include/runway/model.h              legacy 10-flight domain
include/runway/scheduler.h          legacy CP-SAT API
include/runway/joint.h              new 20-flight domain + public optimizer API
src/sample_problem.cpp              legacy fixture
src/validator.cpp                   legacy validator
src/cp_sat_scheduler.cpp            legacy CP-SAT model
src/joint/sample_joint_problem.cpp  new 20-flight scenario generator
src/joint/joint_validator.cpp       independently checks joint solutions
src/joint/cp_sat_joint.cpp          option, time, runway, fix and cross-conflict model
app/joint_main.cpp                  new runway_demo executable
app/main.cpp                        preserved runway_legacy_demo
src/joint/                         joint implementation
CMakeLists.txt                      C++11 core / C++20 adapter
CMakePresets.json                   optional local build preset
tests/                              core and solver integration tests
```

## 5. Extend safely

- `JointProblem` and `FlightOption` contain only plain data; OR-Tools types
  never leak into these public C++11 headers.
- Change input scenarios in `src/joint/sample_joint_problem.cpp` for now.
  Config-driven JSON replay is a separate milestone, not implemented here.
- Every new business hard rule must be added to both the model compiler
  and the independent validator, and covered by a negative test.
- Avoid treating every arrival/departure globally as mutually exclusive;
  local circuits reflect only the physical resource in question.
- Same-type predecessor checks are separate from mixed runways by design.
- For 140 flights, profile model-building time and resource arc counts
  before trying to optimize search heuristics. This prototype is **not**
  a demonstrated 140-flight production-performance solution.

## 6. Validation in the artifact environment

The C++11 `runway_core`, `runway_core_test` and `runway_joint_core_test` were
compiled and **2/2 core CTest checks passed** with the OR-Tools-disabled
configuration. The execution environment for this generated artifact did not
provide an installed OR-Tools C++ SDK, so the **new C++20 solver adapter and
20-flight solution were not compiled/solved here**. Compile and run all CTest
checks on your existing `/usr/local/ortools-9.14` machine. This verification
limit is intentional and should not be confused with a successful full build.

The OR-Tools C++ APIs used were cross-checked against the upstream
[v9.14 cp_model.h](https://github.com/google/or-tools/blob/v9.14/ortools/sat/cp_model.h),
particularly `AddCircuitConstraint`, `AddArc`, `AddExactlyOne`, conditional
linear constraints and `AddMaxEquality`.
