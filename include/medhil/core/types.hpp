#pragma once
#include <Eigen/Geometry>
#include <cstdint>
#include <variant>

namespace medhil {
using Time = std::int64_t; // simulated microseconds
using Vec = Eigen::Vector3d;
using Joints = Eigen::Matrix<double, 6, 1>;
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Rigid = Eigen::Isometry3d;
enum class Frame { PatientCT, OpticalTracker, RobotBase, ToolTip };
struct Pose {
    Frame from, to;
    Vec translation_m;
    Eigen::Quaterniond rotation;
    Time timestamp_us;
    Rigid transform() const { return Eigen::Translation3d(translation_m) * rotation; }
};
inline Pose pose(const Rigid &t, Frame from, Frame to, Time stamp = 0) {
    return {from, to, t.translation(), Eigen::Quaterniond(t.linear()), stamp};
}
enum class Error { InvalidInput, Degenerate, PoorFit, Unreachable };
template <class T> using Result = std::variant<T, Error>;
} // namespace medhil
