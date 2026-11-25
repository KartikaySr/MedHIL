#pragma once
#include "medhil/core/types.hpp"
namespace medhil {
struct JointState {
    Joints q, velocity, torque;
};
struct Observation {
    Pose tool;
    bool visible;
};
struct Wrench {
    Vec force, torque;
};
struct IActuator {
    virtual ~IActuator() = default;
    virtual JointState read_state() const = 0;
    virtual void command_velocity(const Joints &) = 0;
    virtual void command_torque(const Joints &) = 0;
    virtual void engage_brake() = 0;
    virtual bool is_healthy() const = 0;
};
struct IOpticalTracker {
    virtual ~IOpticalTracker() = default;
    virtual Observation read_tool_pose(Time) const = 0;
};
struct IForceSensor {
    virtual ~IForceSensor() = default;
    virtual Wrench read_wrench(Time) const = 0;
};
} // namespace medhil
