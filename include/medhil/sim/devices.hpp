#pragma once
#include "medhil/hal/devices.hpp"
#include "medhil/kinematics/arm.hpp"
#include <random>
#include <vector>
namespace medhil {
struct DriveConfig {
    unsigned delay_ticks = 2;
    double inertia = .12, damping = .6, max_torque = 4, max_speed = 1.2, encoder_sigma = 1e-6;
};
enum class DriveFault { None, StuckJoint, Runaway, EncoderDropout };
class SimActuator final : public IActuator {
  public:
    SimActuator(Joints initial, DriveConfig config, std::uint64_t seed);
    JointState read_state() const override { return measured_; }
    void command_velocity(const Joints &v) override { command_ = {v, true}; }
    void command_torque(const Joints &t) override { command_ = {t, false}; }
    void engage_brake() override;
    bool is_healthy() const override { return fault_ == DriveFault::None; }
    void inject(DriveFault f) { fault_ = f; }
    void advance();
    const Joints &position() const { return actual_.q; }
    bool braked() const { return brake_; }
    void release_brake() { brake_ = false; }

  private:
    struct Command {
        Joints value = Joints::Zero();
        bool velocity = true;
    };
    DriveConfig cfg_;
    JointState actual_, measured_;
    std::mt19937_64 rng_;
    std::vector<Command> queue_;
    size_t cursor_ = 0;
    Command command_;
    bool brake_ = false;
    DriveFault fault_ = DriveFault::None;
};
struct Window {
    Time start = 0, duration = 0;
    bool active(Time t) const { return t >= start && t - start < duration; }
};
class SimTracker final : public IOpticalTracker {
  public:
    explicit SimTracker(std::uint64_t seed) : rng_(seed) {}
    void sample(Time now, const Rigid &tool_in_tracker);
    Observation read_tool_pose(Time now) const override;
    Window occlusion, stale;
    double position_sigma = .0001, angle_sigma = .0005;
    Time max_jitter_us = 0;

  private:
    Observation sample_{pose(Rigid::Identity(), Frame::ToolTip, Frame::OpticalTracker), false};
    std::mt19937_64 rng_;
    Time frame_ = 0, next_ = 0;
};
class SimForce final : public IForceSensor {
  public:
    explicit SimForce(std::uint64_t seed) : rng_(seed) {}
    void sample(Time now, double clearance, const Vec &normal);
    Wrench read_wrench(Time) const override { return sample_; }
    Window overload;
    double drift_n_per_s = .001;

  private:
    Wrench sample_{Vec::Zero(), Vec::Zero()};
    std::mt19937_64 rng_;
};
} // namespace medhil
