# Design decisions

This page records the **historical V1–V12 decisions**. For subsequent tyre-model
experiments, Ackermann, sensing, estimation and complete-loop integration, read
the [post-V12 decision ledger](Post_V12_Decisions.md),
[technical report](../Dynamics_Development.md) and
[Q/failure analysis](../Estimator_Q_and_Failure_Analysis.md).

[← Documentation](../README.md) · [Version history](../Versions/README.md) · [Results](../../Results/README.md)

## From chassis motion to individual contacts

A planar chassis with X, Y, yaw and corresponding velocities was retained to compare kinematic and dynamic responses. Per-wheel forces, loads and moments prepare future differential braking without confusing four contacts with four simulated wheel angular velocities. Telemetry makes saturation observable beyond a position trace.

## Replace independent limits with combined adhesion

Limiting Fx and Fy independently can spend the same contact capacity twice. V3 introduces a shared friction circle. Requested and transmitted forces are recorded separately: increasing the former does not guarantee an increase in the latter. V4 adds quasi-static load transfer and load-dependent stiffness so that wheel capacities differ during manoeuvres.

## Excess throttle and lateral-capacity loss

V5 reduces rear driven-wheel lateral capacity when longitudinal demand exceeds available capacity. Memory states and exponential transitions represent rapid loss and slower recovery, without adding angular velocities or wheel torques. V6 projects velocities into individual wheel coordinates. This supports instability tests but does not establish a calibrated tyre or a physical traction controller.

## Braking: saturation, approximate lock and recovery

The first acceleration/braking straight showed that saturation alone could produce nearly constant deceleration. A more curved velocity trace is not itself proof of realism: nearly constant braking force can reasonably produce a linear velocity segment. The limit is not always g; mu, loads and resistance matter.

A second mode was introduced to study reduced braking effectiveness under excessive demand. Demand relative to nominal capacity triggers it, with hysteresis separating it from normal operation. Capacity decreases with speed through an exponential function with a lower bound. Written as `mu_eff=mu_base-[mu_base*(1-eta)]`, it subtracts a growing, bounded loss. It was not adopted as a universal tyre law.

Partial relief was distinguished from **fully releasing the brake**. Full release sets braking demand to zero, leaves the mode and rapidly restores the capacity memory. With tau=0.01 s, the equation recovers about 95% in 30 ms and 99.3% in 50 ms. The release test checks this mathematical recovery; it does not model the mechanical spin-up time of a locked wheel.

## Loss of steering in lock mode

The shared circle reduces Fy as longitudinal demand consumes capacity, but does not necessarily force Fy to zero. The special mode therefore uses a zero lateral factor and a unit longitudinal factor applied to the already-reduced capacity. Steering rotation of longitudinal force is also suppressed in that mode, avoiding a residual lateral force from `Fx*sin(delta)`. This is a deliberate approximation, not a complete model of friction under arbitrary sliding.

## A reproducible interface

Previous accelerations and adhesion/lock memories initially entered through additional arguments. Moving them into z produces four inputs `z,u,p,dt` and two outputs `z_next,tele`. The saved comparison verified unchanged trajectories across five manoeuvre families after this reorganization. u still contains total force and steering; independent per-wheel commands are not available yet.

## From isolated manoeuvres to a circuit

Pure Pursuit from coursework practice 4 exposed the plant to inside/outside corners and straights without rewriting its physics. Independent copies protected the originals. Animations use simulated positions and yaw; the velocity arrow transforms vx/vy into global coordinates.

## Plan speed before controlling it

A constant speed reference was unsuitable for comparing straights and corners. V9 constructs a curvature-based profile and propagates acceleration/braking bounds around a closed circuit. The PI follows that profile. A speed cap differs from achieved speed: several high V9 caps yield nearly identical laps because curvature dominates.

## Make the demand expose the limits

V10 expands the track from eight to fourteen corners, adds a chicane and tight 90°/70° corners, and places a 172 m straight before the first major braking event. The driver requests maximum force with rapid exponential application rather than an ideal step. Saturation and approximate locking remain visible instead of being hidden by a stability controller.

## Brake later without changing the plant

V11 raises ayMax from 8 to 11 and axFreno from 8 to 12 m/s², removing 0.10 s of extra anticipation. “Brake less” means a smaller speed reduction approaching corners, not lower maximum brake force. At 35 m/s, planned braking starts 25.5 m later on the first straight. Lap time improves from 51.384 to 47.680 s while approximate lock time rises from 12.276% to 19.869%. A more optimistic plan can work here without being universally feasible.

## Separate 3D visualization from 3D physics

Native graphics were chosen instead of adding a library or a mechanically different vehicle. The body has visual volume and the camera follows yaw, but roll, pitch and wheel rotation are not invented. MP4s render recorded histories; frame rate and the final frame can add a small fraction of a second to their duration.

The video workflow was: calculate/save history, disable the interactive timer, update the scene at regular instants, obtain figure frames with `getframe`, encode MPEG-4 using `VideoWriter`, and check duration/readability with `VideoReader`. The figure and timer are closed afterwards. This renders the application's own figure rather than continuously recording the desktop. Cones can be grouped by colour into fewer graphics objects while preserving geometry; this does not alter simulated states. V11 3D uses 30 fps and V12 uses 15 fps, both at real-time playback speed.

## Extreme acceleration in V12

Maximum driving-force demand rises from 2500 to 5000 N and tauAcel drops from 0.030 to 0.010 s. Braking, mapping, circuit and plant equations remain unchanged. Lap time deteriorates to 149.036 s and maximum error grows to 4.347 m. Rear longitudinal demand exceeds capacity in 98.556% of accelerating samples, versus 49.664% in the baseline. This is a stress condition for future coordination, not a performance improvement.

## Decisions still pending

The next phase keeps the plant frozen and assembles the block architecture. Yaw reference, estimation signals, wheel-force allocation, actuator limits and coordination must be defined. Slip-based ABS/TCS still require wheel angular speed, radius/inertia and a force–slip law. Slip-angle convention, lock-mode assumptions and adhesion parameters need review/calibration before conclusions are extended to a real vehicle.


[27 November: modulated ABS, working Q, research and validation](Modulated_ABS_2026-11-27.md).


**33-corner complete loop, 28 November:** sensors/EKF and ABS/TC/ESC complete in 191.074 s; +0.56% versus perfect state. [Integration, results and academic scope](Extreme_33_Corners_2026-11-28.md).
