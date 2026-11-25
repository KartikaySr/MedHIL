#pragma once
#include "medhil/kinematics/arm.hpp"
#include "medhil/safety/safety.hpp"
namespace medhil {
struct Cycle {
    Inputs inputs;
    Joints command;
    State state;
    Reason reason;
};
class Controller {
  public:
    Controller(IActuator &drive, IOpticalTracker &tracker, IForceSensor &force,
               Rigid tracker_to_base, std::vector<Capsule> fixtures, Limits limits = {});
    Cycle tick(Time now, const Joints &target);
    Supervisor supervisor;
    double separation(const Rigid &tool) const;
    double warning_margin_m = .001;

  private:
    IActuator &drive_;
    IOpticalTracker &tracker_;
    IForceSensor &force_;
    Rigid tracker_to_base_;
    std::vector<Capsule> fixtures_;
    Arm arm_;
};
} // namespace medhil
