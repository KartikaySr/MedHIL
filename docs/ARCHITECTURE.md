# MedHIL architecture

## Frames and registration

A pose maps coordinates from its named source frame into its destination frame. Translations use metres, joint positions use radians and timestamps use simulated microseconds. The tracker reports tool-to-tracker poses. A known tracker-to-base extrinsic places each observation in the robot frame. Paired CT and robot landmarks independently determine the CT-to-base transform used for the target and keep-out geometry.

For centered landmark pairs, registration accumulates the cross-covariance and applies a singular value decomposition. The sign of the last singular direction is chosen to give a proper rotation. Translation aligns the two centroids. Registration requires two independent landmark directions and accepts a fit only if the RMS residual over paired fiducials satisfies the configured millimetre threshold. This residual quantifies fit at the fiducials.

## Motion and geometry

The six standard DH links define the tool pose. Each Jacobian column contains the joint axis crossed with its displacement to the tip, followed by the joint axis. Inverse kinematics iterates a damped least-squares correction with a bounded step, joint clipping and a finite iteration limit. The absolute Jacobian determinant supplies the manipulability index for the square Jacobian.

Fixtures are capsules in the CT frame. Coincident capsule endpoints represent a sphere. Segment-to-segment distance minus fixture radius gives signed clearance for the tool centreline, including its tip. The simulated shaft has a 60 mm centreline with zero radius. The supervisor uses the smaller clearance from encoder-based kinematics and optical observation. The controller scales velocity inside the warning margin. The boundary fault scenario disables that margin to exercise the independent stop path.

## Cycle order and replay

1. Compute plant truth from the current joints. Update the simulated force sensor and any optical frame due on this tick.
2. Read the joint state, optical observation and wrench through the hardware interfaces. The tracker holds its last sample between updates. Frame deadlines follow a 60 Hz rational clock with configurable jitter and are serviced on the first eligible millisecond tick.
3. Calculate the joint servo command and optical position correction. Evaluate the motion envelope, fixture clearance, optical validity, force and numerical guards before issuing the command.
4. Record cycle data, update the actuator through its command-delay ring and integrate velocity and position over one millisecond. A brake latches immediately and clears pending commands.

Each scenario seed initializes a master random engine. Derived seeds drive the devices separately. Monte Carlo runs also perturb the initial joints, tracker noise amplitude and actuator delay. Replay compares complete CSV streams for the same seed and binary. Floating-point libraries and standard-library random distributions may differ across toolchains.

## Supervision and timing

The normal sequence is IDLE → CALIBRATING → READY → TRACKING. Completing motion returns the controller to READY. The supervisor evaluates safety guards in TRACKING and latches the first stop reason. Hardware failures enter FAULT. Visibility loss, data older than 50 ms, a joint envelope violation, fixture contact, force overload, low manipulability and invalid numerical state enter SAFE_STOP. The default force threshold is a simulation parameter of 15 N.

A stop issues the simulated brake in the same tick. The latency reported by the CLI is the interval between the first detected violating sample and the brake command. Sensor sampling delay and physical stopping distance require separate measurements on a physical system. A clear observation and explicit acknowledgement return the supervisor to IDLE. Brake release is a separate actuator operation. Rejected transition requests increment a counter.

## Model scope

The plant uses independent joint inertia and viscous damping. Synthetic contact is a clearance-dependent force along a supplied normal. The simulator runs on a logical clock with synthetic fiducials, targets and obstacles. Physical hardware integration, clinical validation and medical-device certification are outside this release.
