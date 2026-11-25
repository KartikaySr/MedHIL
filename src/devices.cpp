#include "medhil/sim/devices.hpp"
#include <algorithm>
#include <stdexcept>
namespace medhil {
namespace {
Vec noise(std::mt19937_64 &rng, double sigma) {
    std::normal_distribution<double> n;
    Vec result;
    for (int i = 0; i < 3; ++i)
        result[i] = sigma * n(rng);
    return result;
}
} // namespace
SimActuator::SimActuator(Joints q, DriveConfig config, std::uint64_t seed)
    : cfg_(config), actual_{q, Joints::Zero(), Joints::Zero()}, measured_(actual_), rng_(seed),
      queue_(config.delay_ticks) {
    if (!q.allFinite() || !(config.inertia > 0) || !(config.max_torque > 0) ||
        !(config.max_speed > 0))
        throw std::invalid_argument("invalid drive parameters");
}
void SimActuator::engage_brake() {
    brake_ = true;
    command_ = Command{};
    std::fill(queue_.begin(), queue_.end(), Command{});
    actual_.velocity.setZero();
    actual_.torque.setZero();
    measured_ = actual_;
}
void SimActuator::advance() {
    if (brake_)
        return;
    Command applied = command_;
    if (!queue_.empty()) {
        std::swap(applied, queue_[cursor_]);
        cursor_ = (cursor_ + 1) % queue_.size();
    }
    Joints torque = applied.value;
    if (applied.velocity)
        torque = 12 * (applied.value - actual_.velocity);
    actual_.torque = torque.cwiseMax(-cfg_.max_torque).cwiseMin(cfg_.max_torque);
    if (fault_ == DriveFault::Runaway)
        actual_.torque[0] = 2 * cfg_.max_torque;
    actual_.velocity += (.001 / cfg_.inertia) * (actual_.torque - cfg_.damping * actual_.velocity);
    actual_.velocity = actual_.velocity.cwiseMax(-cfg_.max_speed).cwiseMin(cfg_.max_speed);
    if (fault_ == DriveFault::StuckJoint)
        actual_.velocity[0] = 0;
    actual_.q += .001 * actual_.velocity;
    for (int i = 0; i < 6; ++i) {
        if (std::abs(actual_.q[i]) >= Arm::joint_limit) {
            actual_.q[i] = std::copysign(Arm::joint_limit, actual_.q[i]);
            actual_.velocity[i] = 0;
        }
    }
    measured_ = actual_;
    std::normal_distribution<double> n;
    for (int i = 0; i < 6; ++i)
        measured_.q[i] += cfg_.encoder_sigma * n(rng_);
    if (fault_ == DriveFault::EncoderDropout)
        measured_.q[0] = std::numeric_limits<double>::quiet_NaN();
}
void SimTracker::sample(Time now, const Rigid &t) {
    if (now < next_)
        return;
    ++frame_;
    const Time jitter = std::clamp<Time>(max_jitter_us, 0, 8000);
    next_ = frame_ * 1000000 / 60 + std::uniform_int_distribution<Time>(-jitter, jitter)(rng_);
    if (stale.active(now))
        return;
    auto p = pose(t, Frame::ToolTip, Frame::OpticalTracker, now);
    p.translation_m += noise(rng_, position_sigma);
    Vec perturb = noise(rng_, angle_sigma);
    if (perturb.norm() > 0)
        p.rotation = p.rotation * Eigen::AngleAxisd(perturb.norm(), perturb.normalized());
    sample_ = {p, !occlusion.active(now)};
}
Observation SimTracker::read_tool_pose(Time now) const {
    auto obs = sample_;
    obs.visible = obs.visible && !occlusion.active(now);
    return obs;
}
void SimForce::sample(Time now, double clearance, const Vec &normal) {
    const double contact = 300 * std::max(0.0, .003 - clearance) + 1500 * std::max(0.0, -clearance);
    sample_.force = noise(rng_, .01) + contact * normal;
    sample_.force.z() += drift_n_per_s * now / 1e6 + (overload.active(now) ? 30 : 0);
    sample_.torque = noise(rng_, .0001);
}
} // namespace medhil
