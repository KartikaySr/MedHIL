#include "medhil/control/controller.hpp"
namespace medhil {
Controller::Controller(IActuator &drive, IOpticalTracker &tracker, IForceSensor &force,
                       Rigid tracker_to_base, std::vector<Capsule> fixtures, Limits limits)
    : supervisor(limits), drive_(drive), tracker_(tracker), force_(force),
      tracker_to_base_(tracker_to_base), fixtures_(std::move(fixtures)) {}
double Controller::separation(const Rigid &tool) const {
    double minimum = std::numeric_limits<double>::max();
    Vec tip = tool.translation(), proximal = tip - .06 * tool.linear().col(2);
    for (const auto &zone : fixtures_)
        minimum = std::min(minimum, clearance(proximal, tip, zone));
    return minimum;
}
Cycle Controller::tick(Time now, const Joints &target) {
    JointState joints = drive_.read_state();
    Observation obs = tracker_.read_tool_pose(now);
    double distance = 0, dexterity = 0;
    Joints cmd = Joints::Zero();
    if (joints.q.allFinite() && obs.tool.translation_m.allFinite() &&
        obs.tool.from == Frame::ToolTip && obs.tool.to == Frame::OpticalTracker) {
        Pose fk = arm_.forward(joints.q);
        Rigid observed = tracker_to_base_ * obs.tool.transform();
        distance = std::min(separation(fk.transform()), separation(observed));
        Matrix6 j = arm_.jacobian(joints.q);
        dexterity = std::abs(j.determinant());
        // Joint servo with translational optical feedback, bounded in rad/s.
        cmd = (12 * (target - joints.q) +
               2 * j.topRows<3>().transpose() * (fk.translation_m - observed.translation()))
                  .cwiseMax(-.8)
                  .cwiseMin(.8);
        if (distance > 0 && warning_margin_m > 0 && distance < warning_margin_m)
            cmd *= distance / warning_margin_m;
    }
    Inputs data{joints, obs, force_.read_wrench(now), distance, dexterity, drive_.is_healthy()};
    if (!target.allFinite())
        data.manipulability = std::numeric_limits<double>::quiet_NaN();
    supervisor.monitor(now, data, drive_);
    if (supervisor.state() == State::Tracking)
        drive_.command_velocity(cmd);
    else
        cmd.setZero();
    return {data, cmd, supervisor.state(), supervisor.reason()};
}
} // namespace medhil
