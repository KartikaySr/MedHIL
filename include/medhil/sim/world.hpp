#pragma once
#include "medhil/control/controller.hpp"
#include "medhil/registration/registration.hpp"
#include "medhil/sim/devices.hpp"
#include <iosfwd>
namespace medhil {
enum class Scenario { Nominal, Occlusion, Fixture, Force, Joint, Stale, Hardware, MonteCarlo };
struct Outcome {
    State state;
    Reason reason;
    Time violation_us = -1, brake_us = -1;
    bool finite = true, unmitigated = false;
    double final_error_m = 0;
    bool passed(Scenario) const;
};
Outcome run(Scenario scenario, std::uint64_t seed, std::ostream *csv = nullptr,
            unsigned run_id = 0);
} // namespace medhil
