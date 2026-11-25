# Requirements traceability

Status records local CTest results for this implementation. Test names can be passed to CTest with the `-R` filter.

| Requirement ID | Acceptance criterion | Module | GoogleTest case | Status |
| --- | --- | --- | --- | --- |
| REQ-REG-01 | Recover a noiseless rigid transform to less than 1e-9 matrix error | registration | Registration.RigidRecovery | Passed |
| REQ-REG-02 | Return a proper rotation and the expected RMS residual for a reflected tetrahedron | registration | Registration.ReflectionAndResidual | Passed |
| REQ-REG-03 | Reject insufficient, mismatched, nonfinite and collinear data | registration | Registration.InvalidInputs | Passed |
| REQ-KIN-01 | Solve reachable poses to 0.1 mm and 0.01 rad inside joint limits | kinematics | Kinematics.PoseRoundTrips | Passed |
| REQ-KIN-02 | Reject unreachable targets and incorrect destination frames | kinematics | Kinematics.RejectsUnreachableAndWrongFrame | Passed |
| REQ-KIN-03 | Match the six-column analytical Jacobian to finite differences | kinematics | Kinematics.JacobianFiniteDifference | Passed |
| REQ-SIM-01 | Delay commands by the configured tick count, saturate torque and speed, clear commands on brake | actuator | Drive.LimitsAndLatency | Passed |
| REQ-SIM-02 | Enforce joint travel and expose injected torque and encoder failures | actuator | Drive.PositionLimitAndInjectedFailures | Passed |
| REQ-SIM-03 | Produce 60 optical samples per simulated second and report occlusion | tracker | Tracker.CadenceAndVisibility | Passed |
| REQ-SIM-04 | Distinguish free-space noise, contact load and injected overload | force sensor | Force.ContactAndOverload | Passed |
| REQ-SAF-01 | Compute signed shaft clearance to CT-defined spheres and capsules | fixtures | Fixtures.SphereAndCapsule | Passed |
| REQ-SAF-02 | Enforce normal transition table and record rejected transitions | supervisor | Supervisor.TransitionTable | Passed |
| REQ-SAF-03 | Require acknowledgement and clear inputs for recovery | supervisor | Supervisor.RecoveryRequiresClearanceAndAcknowledgement | Passed |
| REQ-SAF-04 | Detect stale timestamps, low manipulability and nonfinite joint data | supervisor | Supervisor.SignalGuards | Passed |
| REQ-VER-01 | Reach a registered target within 0.5 mm and finish in READY | control | Integration.RegisteredTargetTracking | Passed |
| REQ-VER-02 | Brake within two ticks of a detected violation across six fault scenarios | control | Integration.FaultToBrakeLatency | Passed |
| REQ-VER-03 | Retain finite signals through 250 perturbed runs | simulation | Verification.SeededTrialsAndByteReplay | Passed |
| REQ-VER-04 | Record zero unmitigated fixture penetrations across those runs | simulation | Verification.SeededTrialsAndByteReplay | Passed |
| REQ-VER-05 | Replay complete telemetry byte for byte for each of the 250 seeds | telemetry | Verification.SeededTrialsAndByteReplay | Passed |
