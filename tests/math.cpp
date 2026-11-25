#include "medhil/kinematics/arm.hpp"
#include "medhil/registration/registration.hpp"
#include <gtest/gtest.h>
namespace medhil {
const std::vector<Vec> tetra = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
TEST(Registration, RigidRecovery) {
    Rigid truth = Eigen::Translation3d(.1, -.23, .04) * Eigen::AngleAxisd(.6, Vec::UnitY());
    auto target = tetra;
    for (auto &p : target)
        p = truth * p;
    auto r = std::get<Registration>(register_points(tetra, target));
    EXPECT_LT((truth.matrix() - r.ct_to_base.transform().matrix()).norm(), 1e-9);
    EXPECT_LT(r.fre_mm, 1e-9);
}
TEST(Registration, ReflectionAndResidual) {
    auto mirrored = tetra;
    for (auto &p : mirrored)
        p.x() = -p.x();
    auto r = std::get<Registration>(register_points(tetra, mirrored, 1000));
    EXPECT_NEAR(r.ct_to_base.transform().linear().determinant(), 1, 1e-12);
    EXPECT_NEAR(r.fre_mm, 500, 1e-8);
    EXPECT_EQ(std::get<Error>(register_points(tetra, mirrored)), Error::PoorFit);
}
TEST(Registration, InvalidInputs) {
    std::vector<Vec> line = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}};
    EXPECT_EQ(std::get<Error>(register_points(line, line)), Error::Degenerate);
    EXPECT_EQ(std::get<Error>(register_points(line, tetra)), Error::InvalidInput);
    line.resize(2);
    EXPECT_EQ(std::get<Error>(register_points(line, line)), Error::InvalidInput);
    auto invalid = tetra;
    invalid[0].z() = std::numeric_limits<double>::infinity();
    EXPECT_EQ(std::get<Error>(register_points(invalid, tetra)), Error::InvalidInput);
}
TEST(Kinematics, PoseRoundTrips) {
    Arm arm;
    for (int i = -4; i <= 4; ++i) {
        Joints q = home() + i * .08 * Joints::Ones();
        Pose goal = arm.forward(q);
        auto solution = std::get<Joints>(arm.inverse(goal, q + .08 * Joints::Ones()));
        auto actual = arm.forward(solution);
        EXPECT_LT((goal.translation_m - actual.translation_m).norm(), 1e-4);
        EXPECT_LT(goal.rotation.angularDistance(actual.rotation), .01);
        EXPECT_LE(solution.cwiseAbs().maxCoeff(), Arm::joint_limit);
    }
}
TEST(Kinematics, RejectsUnreachableAndWrongFrame) {
    Arm arm;
    auto goal = arm.forward(home());
    goal.translation_m.x() = 5;
    EXPECT_EQ(std::get<Error>(arm.inverse(goal, home())), Error::Unreachable);
    goal.to = Frame::PatientCT;
    EXPECT_EQ(std::get<Error>(arm.inverse(goal, home())), Error::InvalidInput);
}
TEST(Kinematics, JacobianFiniteDifference) {
    Arm arm;
    Joints q = home();
    auto j = arm.jacobian(q);
    for (int i = 0; i < 6; ++i) {
        auto dq = Joints::Unit(i) * 1e-6;
        Pose lo = arm.forward(q - dq), hi = arm.forward(q + dq);
        Eigen::AngleAxisd angle(hi.rotation * lo.rotation.conjugate());
        EXPECT_LT(((hi.translation_m - lo.translation_m) / 2e-6 - j.col(i).head<3>()).norm(), 1e-8);
        EXPECT_LT((angle.axis() * angle.angle() / 2e-6 - j.col(i).tail<3>()).norm(), 1e-8);
    }
}
} // namespace medhil
