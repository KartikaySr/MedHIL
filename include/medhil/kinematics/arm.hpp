#pragma once
#include "medhil/core/types.hpp"
namespace medhil {
class Arm {
  public:
    Pose forward(const Joints &q) const;
    Matrix6 jacobian(const Joints &q) const;
    double manipulability(const Joints &q) const;
    Result<Joints> inverse(const Pose &target, Joints seed) const;
    static constexpr double joint_limit = 3.0;
};
Joints home();
} // namespace medhil
