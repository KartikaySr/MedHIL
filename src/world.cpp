#include "medhil/sim/world.hpp"
#include "medhil/telemetry/csv.hpp"
#include <stdexcept>
namespace medhil {
bool Outcome::passed(Scenario s) const {
    if (!finite || unmitigated)
        return false;
    if (s == Scenario::Nominal || s == Scenario::MonteCarlo)
        return state == State::Ready && final_error_m < .0005;
    Reason expected = Reason::Hardware;
    switch (s) {
    case Scenario::Occlusion:
        expected = Reason::Occluded;
        break;
    case Scenario::Stale:
        expected = Reason::Stale;
        break;
    case Scenario::Force:
        expected = Reason::Force;
        break;
    case Scenario::Joint:
        expected = Reason::JointLimit;
        break;
    case Scenario::Fixture:
        expected = Reason::Fixture;
        break;
    default:
        break;
    }
    return state == (expected == Reason::Hardware ? State::Fault : State::SafeStop) &&
           reason == expected && violation_us >= 0 && brake_us >= violation_us &&
           brake_us - violation_us <= 2000;
}
Outcome run(Scenario s, std::uint64_t seed, std::ostream *csv, unsigned run_id) {
    std::mt19937_64 rng(seed);
    const Rigid ct_truth =
        Eigen::Translation3d(.08, -.03, .06) * Eigen::AngleAxisd(.19, Vec::UnitZ());
    const Rigid tracker_to_base =
        Eigen::Translation3d(-.15, .21, .27) * Eigen::AngleAxisd(-.3, Vec::UnitY());
    std::vector<Vec> ct = {{0, 0, 0}, {.1, 0, 0}, {0, .12, 0}, {0, 0, .08}}, base;
    for (const auto &p : ct)
        base.push_back(ct_truth * p);
    auto calibration = register_points(ct, base);
    if (!std::holds_alternative<Registration>(calibration))
        throw std::runtime_error("registration failed");
    Pose registered = std::get<Registration>(calibration).ct_to_base;
    Arm arm;
    Joints initial = home();
    DriveConfig cfg;
    if (s == Scenario::MonteCarlo) {
        cfg.delay_ticks = std::uniform_int_distribution<unsigned>(1, 5)(rng);
        for (int axis = 0; axis < initial.size(); ++axis)
            initial[axis] += std::uniform_real_distribution<double>(-.04, .04)(rng);
    }
    Joints goal = initial;
    goal[0] += (s == Scenario::MonteCarlo ? .005 : .075);
    auto planned = arm.forward(goal);
    Rigid target_ct = ct_truth.inverse() * planned.transform();
    auto ik = arm.inverse(
        pose(registered.transform() * target_ct, Frame::ToolTip, Frame::RobotBase), initial);
    if (!std::holds_alternative<Joints>(ik))
        throw std::runtime_error("target unreachable");
    Vec center = target_ct.translation();
    if (s != Scenario::Fixture)
        center += Vec(.08, .09, .02);
    Capsule fixture{center, center, .004};
    SimActuator drive(initial, cfg, rng());
    SimTracker tracker(rng());
    SimForce force(rng());
    tracker.max_jitter_us = 350;
    if (s == Scenario::MonteCarlo)
        tracker.position_sigma = std::uniform_real_distribution<double>(.00005, .0004)(rng);
    if (s == Scenario::Occlusion)
        tracker.occlusion = {60000, 90000};
    if (s == Scenario::Stale)
        tracker.stale = {40000, 180000};
    if (s == Scenario::Force)
        force.overload = {60000, 30000};
    Limits limits;
    if (s == Scenario::Joint)
        limits.speed = .25;
    Controller controller(drive, tracker, force, tracker_to_base, {to_base(fixture, registered)},
                          limits);
    // Stress-test boundary crossing with the anticipatory margin disabled.
    if (s == Scenario::Fixture)
        controller.warning_margin_m = 0;
    auto &supervisor = controller.supervisor;
    supervisor.transition(State::Calibrating);
    supervisor.transition(State::Ready);
    supervisor.transition(State::Tracking);
    Outcome out{State::Tracking, Reason::Clear};
    if (csv && run_id == 0)
        csv_header(*csv);
    const int ticks = s == Scenario::MonteCarlo ? 200 : 750;
    for (int k = 0; k < ticks; ++k) {
        Time now = k * 1000LL;
        Rigid truth = arm.forward(drive.position()).transform();
        tracker.sample(now, tracker_to_base.inverse() * truth);
        double actual_clearance = controller.separation(truth);
        force.sample(now, actual_clearance, Vec::UnitZ());
        if (s == Scenario::Hardware && k == 60)
            drive.inject(DriveFault::StuckJoint);
        auto cycle = controller.tick(now, std::get<Joints>(ik));
        auto &x = cycle.inputs;
        if (out.violation_us < 0 && supervisor.evaluate(now, x) != Reason::Clear)
            out.violation_us = now;
        if (out.brake_us < 0 && drive.braked())
            out.brake_us = now;
        out.finite &= x.joints.q.allFinite() && x.joints.velocity.allFinite() &&
                      x.joints.torque.allFinite() && x.wrench.force.allFinite() &&
                      x.wrench.torque.allFinite() && x.optical.tool.translation_m.allFinite() &&
                      x.optical.tool.rotation.coeffs().allFinite() && cycle.command.allFinite() &&
                      std::isfinite(x.distance) && std::isfinite(x.manipulability);
        out.unmitigated |= actual_clearance <= 0 && !drive.braked();
        if (k == ticks - 1 && supervisor.state() == State::Tracking) {
            supervisor.transition(State::Ready);
            drive.engage_brake();
            cycle.state = supervisor.state();
            cycle.command.setZero();
        }
        if (csv)
            csv_row(*csv, run_id, seed, now, cycle);
        if (drive.braked())
            break;
        drive.advance();
    }
    out.state = supervisor.state();
    out.reason = supervisor.reason();
    out.final_error_m =
        (arm.forward(drive.position()).translation_m - planned.translation_m).norm();
    return out;
}
} // namespace medhil
