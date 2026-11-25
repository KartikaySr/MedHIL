#include "medhil/sim/devices.hpp"
#include <gtest/gtest.h>
namespace medhil {
TEST(Drive, LimitsAndLatency) {
    DriveConfig cfg;
    cfg.max_speed = .01;
    SimActuator drive(home(), cfg, 42);
    drive.command_torque(100 * Joints::Ones());
    for (unsigned i = 0; i < cfg.delay_ticks; ++i) {
        drive.advance();
        EXPECT_TRUE(drive.read_state().torque.isZero());
    }
    drive.advance();
    EXPECT_TRUE(drive.read_state().torque.isConstant(cfg.max_torque));
    EXPECT_TRUE(drive.read_state().velocity.isConstant(cfg.max_speed));
    drive.engage_brake();
    auto before = drive.position();
    drive.advance();
    EXPECT_TRUE(before == drive.position());
    drive.release_brake();
    drive.advance();
    EXPECT_TRUE(drive.read_state().torque.isZero());
}
TEST(Drive, PositionLimitAndInjectedFailures) {
    DriveConfig cfg;
    cfg.delay_ticks = 0;
    SimActuator drive(Joints::Constant(Arm::joint_limit), cfg, 11);
    drive.command_velocity(Joints::Ones());
    drive.advance();
    EXPECT_TRUE(drive.position().isConstant(Arm::joint_limit));
    drive.inject(DriveFault::Runaway);
    drive.advance();
    EXPECT_GT(drive.read_state().torque[0], cfg.max_torque);
    drive.inject(DriveFault::EncoderDropout);
    drive.advance();
    EXPECT_FALSE(drive.read_state().q.allFinite());
}
TEST(Tracker, CadenceAndVisibility) {
    SimTracker tracker(42);
    tracker.occlusion = {20000, 10000};
    Time previous = -1;
    int frames = 0;
    for (Time t = 0; t < 1000000; t += 1000) {
        tracker.sample(t, Rigid::Identity());
        auto obs = tracker.read_tool_pose(t);
        if (obs.tool.timestamp_us != previous) {
            if (previous >= 0)
                EXPECT_TRUE(obs.tool.timestamp_us - previous == 16000 ||
                            obs.tool.timestamp_us - previous == 17000);
            ++frames;
            previous = obs.tool.timestamp_us;
        }
        if (t >= 20000 && t < 30000)
            EXPECT_FALSE(obs.visible);
    }
    EXPECT_EQ(frames, 60);
}
TEST(Force, ContactAndOverload) {
    SimForce f(42);
    f.sample(0, 1, Vec::UnitX());
    EXPECT_LT(f.read_wrench(0).force.norm(), .1);
    f.sample(1000, -.01, Vec::UnitX());
    EXPECT_GT(f.read_wrench(1000).force.x(), 15);
    f.overload = {2000, 1000};
    f.sample(2000, 1, Vec::UnitX());
    EXPECT_GT(f.read_wrench(2000).force.z(), 15);
}
} // namespace medhil
