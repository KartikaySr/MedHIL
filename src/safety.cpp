#include "medhil/safety/safety.hpp"
#include <algorithm>
#include <stdexcept>
namespace medhil {
Capsule to_base(const Capsule &c, const Pose &p) {
    if (p.from != Frame::PatientCT || p.to != Frame::RobotBase || !(c.radius >= 0))
        throw std::invalid_argument("invalid fixture transform or radius");
    return {p.transform() * c.a, p.transform() * c.b, c.radius};
}
double clearance(const Vec &start, const Vec &tip, const Capsule &c) {
    // Minimise squared separation of two bounded line segments.
    Vec u = tip - start, v = c.b - c.a, w = start - c.a;
    double a = u.squaredNorm(), b = u.dot(v), d = u.dot(w), e = v.squaredNorm(), f = v.dot(w);
    auto unit = [](double x) { return std::clamp(x, 0.0, 1.0); };
    double s = 0, t = 0;
    if (e == 0)
        s = a > 0 ? unit(-d / a) : 0;
    else {
        double det = u.cross(v).squaredNorm();
        if (det > 0)
            s = unit((b * f - d * e) / det);
        t = unit((b * s + f) / e);
        if (a > 0)
            s = unit((b * t - d) / a);
    }
    return (w + s * u - t * v).norm() - c.radius;
}
const char *name(State s) {
    constexpr const char *names[] = {"IDLE",     "CALIBRATING", "READY",
                                     "TRACKING", "SAFE_STOP",   "FAULT"};
    return names[static_cast<int>(s)];
}
const char *name(Reason r) {
    constexpr const char *names[] = {"clear",       "hardware", "numeric", "occluded", "stale",
                                     "joint_limit", "fixture",  "force",   "singular"};
    return names[static_cast<int>(r)];
}
bool Supervisor::transition(State next) {
    bool allowed = (state_ == State::Idle && next == State::Calibrating) ||
                   (state_ == State::Calibrating && next == State::Ready) ||
                   (state_ == State::Ready && next == State::Tracking) ||
                   (state_ == State::Tracking && next == State::Ready);
    if (allowed)
        state_ = next;
    else
        ++rejected_;
    return allowed;
}
Reason Supervisor::evaluate(Time now, const Inputs &x) const {
    if (!x.healthy)
        return Reason::Hardware;
    if (!x.joints.q.allFinite() || !x.joints.velocity.allFinite() || !x.joints.torque.allFinite() ||
        !x.wrench.force.allFinite() || !x.wrench.torque.allFinite() || !std::isfinite(x.distance) ||
        !std::isfinite(x.manipulability) || !x.optical.tool.translation_m.allFinite() ||
        !(std::abs(x.optical.tool.rotation.norm() - 1) < 1e-8))
        return Reason::Numeric;
    if (!x.optical.visible)
        return Reason::Occluded;
    const Time stamp = x.optical.tool.timestamp_us;
    if (stamp < 0 || stamp > now || now - stamp > limits_.age)
        return Reason::Stale;
    if (x.joints.q.cwiseAbs().maxCoeff() > limits_.angle ||
        x.joints.velocity.cwiseAbs().maxCoeff() > limits_.speed ||
        x.joints.torque.cwiseAbs().maxCoeff() > limits_.torque)
        return Reason::JointLimit;
    if (x.distance <= 0)
        return Reason::Fixture;
    if (x.wrench.force.norm() > limits_.force)
        return Reason::Force;
    if (x.manipulability < limits_.manipulability)
        return Reason::Singular;
    return Reason::Clear;
}
void Supervisor::monitor(Time now, const Inputs &x, IActuator &actuator) {
    if (state_ != State::Tracking)
        return;
    reason_ = evaluate(now, x);
    if (reason_ != Reason::Clear) {
        state_ = reason_ == Reason::Hardware ? State::Fault : State::SafeStop;
        actuator.engage_brake();
    }
}
bool Supervisor::acknowledge_fault(Time now, const Inputs &inputs) {
    if ((state_ != State::Fault && state_ != State::SafeStop) ||
        evaluate(now, inputs) != Reason::Clear) {
        ++rejected_;
        return false;
    }
    state_ = State::Idle;
    reason_ = Reason::Clear;
    return true;
}
} // namespace medhil
