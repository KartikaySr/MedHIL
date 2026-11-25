#pragma once
#include "medhil/control/controller.hpp"
#include <iomanip>
#include <ostream>
namespace medhil {
inline void csv_header(std::ostream &os) {
    os << "run,seed,time_us,state,reason,visible,observation_us,clearance_m,force_norm_n,"
          "manipulability";
    for (const char *field : {"q_rad", "dq_rad_s", "tau_nm", "command_rad_s"})
        for (int i = 1; i <= 6; ++i)
            os << ',' << field << i;
    os << '\n';
}
inline void csv_row(std::ostream &os, unsigned run, std::uint64_t seed, Time t, const Cycle &c) {
    const auto &x = c.inputs;
    os << std::setprecision(17) << run << ',' << seed << ',' << t << ',' << name(c.state) << ','
       << name(c.reason) << ',' << x.optical.visible << ',' << x.optical.tool.timestamp_us << ','
       << x.distance << ',' << x.wrench.force.norm() << ',' << x.manipulability;
    for (const Joints *v : {&x.joints.q, &x.joints.velocity, &x.joints.torque, &c.command})
        for (int axis = 0; axis < v->size(); ++axis)
            os << ',' << (*v)[axis];
    os << '\n';
}
} // namespace medhil
