#include "medhil/kinematics/arm.hpp"
#include <Eigen/Cholesky>
#include <array>
namespace medhil {
namespace {
constexpr double half_pi = 1.5707963267948966;
std::array<Rigid, 7> chain(const Joints &q) {
    constexpr double a[] = {0, .31, .26, 0, 0, 0}, d[] = {.12, 0, 0, .08, .07, .08};
    constexpr double alpha[] = {half_pi, 0, 0, half_pi, -half_pi, 0};
    std::array<Rigid, 7> t;
    t[0] = Rigid::Identity();
    for (int i = 0; i < 6; ++i) {
        t[i + 1] = t[i] * Eigen::AngleAxisd(q[i], Vec::UnitZ()) *
                   Eigen::Translation3d(a[i], 0, d[i]) * Eigen::AngleAxisd(alpha[i], Vec::UnitX());
    }
    return t;
}
} // namespace
Joints home() { return (Joints() << .1, -.6, 1.1, .3, .8, -.2).finished(); }
Pose Arm::forward(const Joints &q) const {
    return pose(chain(q).back(), Frame::ToolTip, Frame::RobotBase);
}
Matrix6 Arm::jacobian(const Joints &q) const {
    const auto t = chain(q);
    Matrix6 j;
    for (int i = 0; i < 6; ++i) {
        Vec axis = t[i].linear().col(2);
        j.col(i).head<3>() = axis.cross(t.back().translation() - t[i].translation());
        j.col(i).tail<3>() = axis;
    }
    return j;
}
double Arm::manipulability(const Joints &q) const { return std::abs(jacobian(q).determinant()); }
Result<Joints> Arm::inverse(const Pose &target, Joints q) const {
    if (target.to != Frame::RobotBase || target.from != Frame::ToolTip || !q.allFinite() ||
        !target.translation_m.allFinite() || !(std::abs(target.rotation.norm() - 1) < 1e-9))
        return Error::InvalidInput;
    q = q.cwiseMax(-joint_limit).cwiseMin(joint_limit);
    for (int i = 0; i < 100; ++i) {
        Pose now = forward(q);
        Eigen::AngleAxisd delta(target.rotation * now.rotation.conjugate());
        Joints e;
        e << target.translation_m - now.translation_m, delta.axis() * delta.angle();
        if (e.norm() < 1e-7)
            return q;
        Matrix6 j = jacobian(q);
        Joints step =
            j.transpose() * (j * j.transpose() + 1e-5 * Matrix6::Identity()).ldlt().solve(e);
        q = (q + step / std::max(1.0, step.norm() / 0.25))
                .cwiseMax(-joint_limit)
                .cwiseMin(joint_limit);
    }
    return Error::Unreachable;
}
} // namespace medhil
