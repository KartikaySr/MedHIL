#include "medhil/safety/safety.hpp"
#include "medhil/sim/devices.hpp"
#include <gtest/gtest.h>
namespace medhil {
TEST(Fixtures, SphereAndCapsule) {
    Capsule sphere{Vec::Zero(), Vec::Zero(), 1};
    EXPECT_DOUBLE_EQ(clearance({2, 0, 0}, {1, 0, 0}, sphere), 0);
    EXPECT_DOUBLE_EQ(clearance({-2, 0, 0}, {2, 0, 0}, sphere), -1);
    Capsule capsule{{0, 0, -1}, {0, 0, 1}, .25};
    EXPECT_DOUBLE_EQ(clearance({0, 0, 2}, {0, 0, 2}, capsule), .75);
    EXPECT_DOUBLE_EQ(clearance({-1, 0, 0}, {1, 0, 0}, capsule), -.25);
    EXPECT_DOUBLE_EQ(clearance({1, 0, -2}, {1, 0, 2}, capsule), .75);
    Pose transform = pose(Rigid(Eigen::Translation3d(1, 0, 0)), Frame::PatientCT, Frame::RobotBase);
    EXPECT_NEAR(clearance({1, 0, 0}, {1, 0, 0}, to_base(capsule, transform)), -.25, 1e-12);
}
Inputs good() {
    return {{home(), Joints::Zero(), Joints::Zero()},
            {pose(Rigid::Identity(), Frame::ToolTip, Frame::OpticalTracker), true},
            {Vec::Zero(), Vec::Zero()},
            .02,
            .1,
            true};
}
Supervisor at(State state, SimActuator &drive) {
    Supervisor s;
    if (state == State::Idle)
        return s;
    s.transition(State::Calibrating);
    if (state == State::Calibrating)
        return s;
    s.transition(State::Ready);
    if (state == State::Ready)
        return s;
    s.transition(State::Tracking);
    if (state == State::Tracking)
        return s;
    auto data = good();
    data.optical.visible = false;
    data.healthy = state != State::Fault;
    s.monitor(0, data, drive);
    return s;
}
TEST(Supervisor, TransitionTable) {
    const State states[] = {State::Idle,     State::Calibrating, State::Ready,
                            State::Tracking, State::SafeStop,    State::Fault};
    for (auto start : states)
        for (auto dest : states) {
            SimActuator drive(home(), {}, 0);
            auto supervisor = at(start, drive);
            bool allowed = (start == State::Idle && dest == State::Calibrating) ||
                           (start == State::Calibrating && dest == State::Ready) ||
                           (start == State::Ready && dest == State::Tracking) ||
                           (start == State::Tracking && dest == State::Ready);
            EXPECT_EQ(supervisor.transition(dest), allowed);
            EXPECT_EQ(supervisor.state(), allowed ? dest : start);
            EXPECT_EQ(supervisor.rejected(), allowed ? 0U : 1U);
        }
}
TEST(Supervisor, RecoveryRequiresClearanceAndAcknowledgement) {
    for (auto stopped : {State::SafeStop, State::Fault}) {
        SimActuator drive(home(), {}, 4);
        auto supervisor = at(stopped, drive);
        auto data = good();
        data.distance = 0;
        EXPECT_FALSE(supervisor.acknowledge_fault(0, data));
        supervisor.monitor(0, good(), drive);
        EXPECT_EQ(supervisor.state(), stopped);
        EXPECT_TRUE(supervisor.acknowledge_fault(0, good()));
        EXPECT_EQ(supervisor.state(), State::Idle);
    }
}
TEST(Supervisor, SignalGuards) {
    Supervisor s;
    auto x = good();
    EXPECT_EQ(s.evaluate(50000, x), Reason::Clear);
    EXPECT_EQ(s.evaluate(50001, x), Reason::Stale);
    x.manipulability = 0;
    EXPECT_EQ(s.evaluate(0, x), Reason::Singular);
    x.joints.q.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(s.evaluate(0, x), Reason::Numeric);
}
} // namespace medhil
