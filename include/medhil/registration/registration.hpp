#pragma once
#include "medhil/core/types.hpp"
#include <vector>
namespace medhil {
struct Registration {
    Pose ct_to_base;
    double fre_mm;
};
Result<Registration> register_points(const std::vector<Vec> &ct, const std::vector<Vec> &base,
                                     double max_fre_mm = 1.0);
} // namespace medhil
