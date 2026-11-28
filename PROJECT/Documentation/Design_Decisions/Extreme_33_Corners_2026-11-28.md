# Extreme circuit, ESC integration and complete-loop interpretation

[Results](../../Results/Extreme_33_Corners/README.md) · [Runnable harness](../../Current%20State/Ensayos_circuito_extremo/README.md)

## Keep the physical model fixed and expose control limits

The new circuit has 33 curves over 4,908.2 m: linked fast bends, chicanes, hairpins, tight direction reversals and long straights. Tangent-connected straight/arc primitives preserve continuity. Reference speed is capped at 40 m/s with the same demanding curvature plan in each comparison. Longitudinal requests retain the rapid existing ramps and 5,000 N limits. Pure Pursuit retains `Ld0=1`, `kLd=0.15`; the shorter experimental moose-test setting is not used.

The existing force-based TC and modulated ABS remain the academic design, not unfinished production slip regulators. The eighteen dynamics/control/configuration/sensing/estimation files are byte-identical to the previous current package. Existing fourteen-corner and moose benchmarks remain distinct instead of being silently overwritten.

## Coordinate one final ABS pass

ESC V1 originally applied ABS internally to its own selective request, while the moose driver could not request ordinary braking. A circuit needs normal braking too. The new external `solicitarCorreccionESC` exposes the same reference, thresholds, gains, wheel choice and force ramps before ABS. `coordinarAyudasExtremo` combines nominal and selective braking and applies one final ABS call. This avoids two independent modulators in series or dropping the driver's braking request. Its pre-ABS ESC law was compared with the original controller.

The plant rejects simultaneous positive/negative wheel demands. Motor demand is therefore removed while selective ESC braking is active; rear driving forces remain equal. The longitudinal integrator is frozen and its actuator memory cleared during this removal to avoid accumulating unused driving demand. This is external coordination, not a changed ESC yaw law. No active steering correction has been added.

## Separate failure observation from driver recovery

The four-case perfect-state test terminates on severe loss of control and preserves all failures. A separate visualization compares ABS+TC with and without ESC while explicitly adding a temporary driver recovery manoeuvre. The original aggressive plan resumes after each recovery. The no-ESC completion time includes repeated off-track excursions; it is not a legal racing lap or a pure ESC performance comparison. No state is teleported, restarted or clamped to a target trajectory.

## Complete the estimated loop without reducing measurement noise

The same 33-corner scenario was then connected to the current sensors and EKF. Planning uses estimated position and the controls use estimated motion plus controller-owned capacity memories; true tyre memories are not measurements. Q remains `diag([.001 .001 .00001 .003 .003].^2)`, with GPS 0.5 m, x/y acceleration 0.2 m/s², gyro 0.01 rad/s and steering 0.1 degrees. Seed 1 and existing initialization are retained. No obstacles are present, so the emergency path is inactive.

The full loop completes in 191.074 s versus 190.006 s with perfect state (+0.56%). Maximum actual path error increases by 7.46 cm to 0.504636 m; position estimation RMSE is 0.153999 m. ESC intervention time rises by 4.162 s (+19.93%), not necessarily the same percentage of energy or force. Both noise and replacement of true capacity inputs with their internal prediction contribute to this comparison. Repeating random seeds and targeting individual manoeuvres are still needed for robustness evidence.

## Interpret force gaps correctly

In traction, the force request is converted to motor torque `T=R*F_req`; wheel acceleration satisfies `I*omega_dot=T-R*Fx`, and contact Fx comes from longitudinal Pacejka. The plant subsequently limits total motor power to 60 kW. Thus a request can exceed the sustained motor force, approximately `P/v` without wheel acceleration or slip. This is about 1,500 N at 40 m/s. Wheel inertia adds transient response; it is not a universal transport delay. In braking, combined lateral demand can scale Fx through the existing friction circle even after ABS limits longitudinal demand. Telemetry separates nominal, aid-corrected and transmitted forces; adding those in wheel coordinates is not the same as net chassis force.

## Defensible conclusion and academic scope

> The model qualitatively reproduces the main adhesion-loss phenomena and the coordinated action of the driving aids. Its results are physically plausible within the adopted assumptions, while quantitative agreement with a particular real vehicle would require parameter identification and experimental validation.

Experimental tyre identification and replacing force-based ABS/TC with production wheel-slip regulation are outside the stated academic scope. They are limitations, not required unfinished implementation. Remaining work is targeted ABS/TC/ESC validation, multiple noise seeds and estimation consistency, emergency scenarios, reproducible final packaging and the project report/presentation. A successful simulated lap alone does not validate real-world vehicle dynamics.
