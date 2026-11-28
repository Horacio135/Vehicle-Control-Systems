# Decisions after V12

[← Design history](README.md) · [Technical development](../Dynamics_Development.md) · [Current package](../../Current%20State/README.md) · [Results](../../Results/Closed_Loop/README.md)

This ledger records the decisions that led from the V12 stress scenario to the current complete loop. Rejected experiments remain evidence; they are not active default models.

| Decision | Reason | Consequence and limit |
|---|---|---|
| Keep requested and transmitted forces separate | An arbitrarily large motor/brake request must not create unlimited contact force | Telemetry preserves both demand and transmission |
| Reject longitudinal multiplication by the old lateral eta as the final law | Persistent excess demand did not explain wheel-speed evolution or mechanical recovery | Retain the test, introduce rotational driving slip |
| Introduce longitudinal Magic Formula and wheel inertia | Force–slip peak/post-peak behaviour needs a slip variable and torque balance | Illustrative radius, inertia, power and coefficients still need identification |
| Discard the full combined Pacejka V2 configuration | The uncalibrated coupling/unchanged force controller combination failed the circuit | Does not establish that pure lateral Pacejka is unsuitable |
| Select rear-traction-only Pacejka with historical braking | Preserve the chosen driving response while restoring the requested braking model | A declared hybrid, without complete rotational continuity at branch switching |
| Apply Ackermann and correct double steering in historical braking alpha | Parallel front angles and duplicated coordinate rotation were inconsistent | Shared turning geometry; actual dynamic slip is still permitted |
| Add ay and equivalent steering to sensors | Lateral manoeuvres require more than longitudinal acceleration and GPS | No fictional sensors for forces, friction or internal tyre memories |
| Extend the P5 EKF while retaining prediction/correction structure | vy=0 and longitudinal-only position equations fail during drift | Seven-state filter; no new bias observer or kinematic no-slip constraint |
| Keep tyre-memory calculations inside a controller-owned model | A controller must not read hidden physical tyre states as measurements | Memories receive no direct measurement noise; uncertainty arrives through inputs/assumptions |
| Close the loop with external adapters | Existing block internals were to remain unchanged without prior review | Thirteen block functions preserved byte-for-byte at loop closure |
| Retune residual Q in configuration | Inherited residual uncertainty was inappropriate once IMU uncertainty was explicitly propagated | Manual provisional tuning; same-measurement replay measures its effect |
| Query the speed profile using estimated position | True position in planning would leave a hidden perfect-state path | Truth is confined to sensor synthesis, initialization and evaluation |
| Retain the failed run and uncontrolled excursions | A successful controlled lap alone hides important failure modes | Videos, histories and chronology show both successes and limitations |

## Evidence does not support stronger claims

The current ABS/TC are force limiters, not physical-slip feedback control. ESC V1 has since been implemented and connected to the [33-corner estimated loop](../../Results/Extreme_33_Corners/README.md); there is still no evasive steering planner or experimentally calibrated combined tyre. The first uncontrolled departure precedes large estimator divergence, while later vx-clamping exposes a plant/estimator mismatch. Neither “the estimator alone caused the first departure” nor “the estimator is validated because ABS/TC keep the car on track” follows from the records.

## Review before the next internal changes

Review the no-reverse constraint mismatch, covariance consistency during extreme motion and the available measurements before changing physics or estimator equations. Separate TC-only and ABS-only experiments, further random seeds, outages and same-measurement replay are appropriate next checks. Steering-actuator limits and simultaneous driving/selective-braking actuation remain limitations. The current ESC removes motor drive during selective braking to satisfy the existing plant contract. See the [33-corner decisions](Extreme_33_Corners_2026-11-28.md) for the completed integration and academic scope.
