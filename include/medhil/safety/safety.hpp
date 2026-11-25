#pragma once
#include "medhil/hal/devices.hpp"
#include <vector>
namespace medhil {
// A sphere has coincident endpoints. All coordinates use metres.
struct Capsule {
    Vec a, b;
    double radius;
};
Capsule to_base(const Capsule &, const Pose &ct_to_base);
double clearance(const Vec &shaft_start, const Vec &tip, const Capsule &);
enum class State { Idle, Calibrating, Ready, Tracking, SafeStop, Fault };
enum class Reason {
    Clear,
    Hardware,
    Numeric,
    Occluded,
    Stale,
    JointLimit,
    Fixture,
    Force,
    Singular
};
const char *name(State);
const char *name(Reason);
struct Inputs {
    JointState joints;
    Observation optical;
    Wrench wrench;
    double distance, manipulability;
    bool healthy;
};
struct Limits {
    double angle = 2.98, speed = 1.21, torque = 4.01, force = 15, manipulability = 1e-6;
    Time age = 50000;
};
class Supervisor {
  public:
    explicit Supervisor(Limits limits = {}) : limits_(limits) {}
    bool transition(State);
    bool acknowledge_fault(Time, const Inputs &);
    Reason evaluate(Time, const Inputs &) const;
    void monitor(Time, const Inputs &, IActuator &);
    State state() const { return state_; }
    Reason reason() const { return reason_; }
    unsigned rejected() const { return rejected_; }

  private:
    State state_ = State::Idle;
    Reason reason_ = Reason::Clear;
    unsigned rejected_ = 0;
    Limits limits_;
};
} // namespace medhil
