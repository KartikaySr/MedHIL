# MedHIL

**Deterministic simulation and software verification for image-guided surgical robotics**
Kartikay Srivastava · C++17 · Eigen · CMake · GoogleTest

When an optical tracker loses sight of a moving surgical tool, how many control cycles elapse before the brake command? MedHIL makes that question reproducible. It registers synthetic CT landmarks to a six-joint robot, tracks a target, injects faults and records the supervisor's decision on every simulated millisecond.

## First run

On macOS with Apple command line tools installed, install CMake with `brew install cmake`. On Ubuntu, install the build tools with `sudo apt-get install build-essential cmake git`. The first configuration uses an installed Eigen package when available or fetches Eigen 3.4.0. Test builds fetch GoogleTest v1.14.0.

```sh
git clone https://github.com/KartikaySr/MedHIL.git
cd MedHIL
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
./build/medhil_cli --scenario occlusion_fault --seed 42 --output occlusion.csv
./build/medhil_cli --scenario monte_carlo --runs 250 --seed 42
```

For a sanitizer build on either platform:

```sh
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG="-O1 -g" -DMEDHIL_ENABLE_SANITIZERS=ON
cmake --build build-debug --parallel
ctest --test-dir build-debug --output-on-failure
```

## Recorded scenarios

Apple Silicon, Release build, seed 42. The timing measures the interval from a detected violation to a brake command on the simulation clock.

| CLI scenario | Terminal state | Observed result |
| --- | --- | --- |
| nominal | READY | Final tip error below 0.01 mm on this synthetic target |
| occlusion_fault | SAFE_STOP | Brake on the first invisible sample at 60 ms |
| stale_tracker | SAFE_STOP | Brake on the first sample older than 50 ms |
| fixture_violation | SAFE_STOP | Brake on detection of a keep-out boundary breach |
| force_overload | SAFE_STOP | Brake on the injected overload at 60 ms |
| joint_limit | SAFE_STOP | Brake when measured velocity exceeds the scenario envelope |
| hardware_fault | FAULT | Brake on an injected stuck-joint fault at 60 ms |
| monte_carlo | READY | 250/250 trials passed with seeds 42–291 |

All six fault runs issued the brake in the detection cycle. CSV rows include observation age inputs, supervisor state and reason, signed fixture clearance, force magnitude, joint measurements and commanded velocity. The CLI returns a nonzero status for a failed scenario or an invalid request.

## Architecture

```text
 CT landmarks + paired robot landmarks
                  |
             registration -----> target and fixtures in robot base
                                      |
                                      v
 synthetic plant <---- device HAL ---> controller + safety supervisor ---> CSV
 arm / optical tracker / wrench         ^           ^
                                       |           |
                                  kinematics   shaft clearance
```

The controller runs at 1 kHz of simulated time. The optical tracker supplies timestamped poses at 60 Hz. Each actuator and sensor uses a seeded random stream. The test suite compares full telemetry across repeated runs under the same build. This release uses simulated devices and synthetic anatomy. The [architecture notes](docs/ARCHITECTURE.md) describe frame conventions, numerical methods, model assumptions and the brake semantics.

## Requirements and tests

The suite contains 17 GoogleTest cases. Several cases cover multiple related requirements. The [traceability document](docs/REQUIREMENTS_TRACEABILITY.md) indexes the same acceptance criteria by requirement ID.

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

## Architectural References

MedHIL is an original C++17 implementation. Three projects informed its high-level design.

- **libfranka** (Franka Robotics) informed the control interfaces, RAII ownership and structured state and error types.
- **PlusLib** (Plus Toolkit) informed the named medical coordinate frames, simulated optical tracking workflow and fiducial-based registration.
- **AMBF** (WPI AIM Lab) informed the separation of actuator and sensor simulation from control logic.
