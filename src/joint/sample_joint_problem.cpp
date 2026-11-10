#include "runway/joint.h"

#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace runway {
namespace {
std::string name(char kind, int number) {
    std::ostringstream os;
    os << kind << std::setfill('0') << std::setw(2) << number;
    return os.str();
}

FlightOption arrival_option(const std::string& option_id) {
    FlightOption o;
    o.id = option_id;
    if (option_id == "P1") {
        o.runway = "R1"; o.star = "STAR-N"; o.fix = "X";
        o.entry_to_fix_min_sec = 120; o.entry_to_fix_max_sec = 180;
        o.fix_to_land_min_sec = 180; o.fix_to_land_max_sec = 240;
    } else if (option_id == "P2") {
        o.runway = "R2"; o.star = "STAR-S"; o.fix = "Y";
        o.entry_to_fix_min_sec = 180; o.entry_to_fix_max_sec = 240;
        o.fix_to_land_min_sec = 210; o.fix_to_land_max_sec = 270;
    } else {
        o.runway = "R2"; o.star = "STAR-E"; o.fix = "X";
        o.entry_to_fix_min_sec = 150; o.entry_to_fix_max_sec = 220;
        o.fix_to_land_min_sec = 210; o.fix_to_land_max_sec = 260;
    }
    return o;
}

FlightOption departure_option(const std::string& option_id) {
    FlightOption o;
    o.id = option_id;
    o.runway = option_id == "Q1" ? "R1" : "R2";
    o.taxi_min_sec = option_id == "Q1" ? 180 : 240;
    return o;
}
} // namespace

JointProblem make_joint_problem(const std::string& scenario) {
    if (scenario != "baseline20" && scenario != "route-switch" &&
        scenario != "fix-conflict" && scenario != "infeasible") {
        throw std::invalid_argument("unknown joint scenario: " + scenario);
    }

    JointProblem p;
    p.horizon_sec = 5400;
    p.runways = {"R1", "R2"};
    p.fixes = {"X", "Y"};
    p.runway_separation = SeparationMatrix{120, 120, 90, 100};
    p.fix_gap_sec = scenario == "fix-conflict" ? 180 : 90;

    for (int i = 1; i <= 10; ++i) {
        JointFlight f;
        f.id = name('A', i);
        f.movement = Movement::Arrival;
        f.input_time_sec = 300 + (i - 1) * 250;
        f.reference_sec = 600 + (i - 1) * 250;
        f.options.push_back(arrival_option("P1"));
        f.options.push_back(arrival_option("P2"));
        if (i > 1) f.options.push_back(arrival_option("P3"));
        if (i == 1 && scenario != "route-switch") {
            // Purposefully prevents P1 from coexisting with D01's NTOT.
            f.options.front().operation_windows.push_back(TimeWindow{600, 650});
        }
        p.flights.push_back(f);
    }
    for (int i = 1; i <= 10; ++i) {
        JointFlight f;
        f.id = name('D', i);
        f.movement = Movement::Departure;
        f.input_time_sec = 450 + (i - 1) * 200;
        f.reference_sec = 630 + (i - 1) * 200;
        f.options.push_back(departure_option("Q1"));
        if (i > 1) f.options.push_back(departure_option("Q2"));
        if (i == 1) {
            f.hard_operation_windows.push_back(
                scenario == "infeasible" ? TimeWindow{500, 600}
                                         : TimeWindow{630, 690});
        }
        if (i == 4) f.hard_operation_windows.push_back(TimeWindow{1230, 1330});
        if (i == 7) f.hard_operation_windows.push_back(TimeWindow{1830, 1930});
        p.flights.push_back(f);
    }
    // Cross-runway route-dependent test: A03/P3 uses R2 while D03/Q1 uses R1.
    p.cross_conflicts.push_back(
        CrossConflict{"A03", "P3", "D03", "Q1", 80, 100});
    return p;
}
} // namespace runway
