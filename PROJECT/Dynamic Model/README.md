# Historical dynamic-model development

[← Project](../README.md) · [Manoeuvres](Ensayos/README.md) · [Design decisions](../Documentation/Design_Decisions/README.md)

**Historical V8 reference:** `CuatroruedasV8_bloqueo_ruedas.mlx`. The active plant is now in [Current State](../Current%20State/README.md): rear traction-only Pacejka with rotational dynamics, restored historical braking and ideal Ackermann. The interface and limitations below describe V8, not the later hybrid selection.

Read [the full theoretical development](../Documentation/Dynamics_Development.md), [post-V12 milestones](../Documentation/Versions/After_V12/README.md) and [archived model alternatives](Archived%20Models/README.md) for the subsequent decisions and rejected combined-slip model.

The prototypes and V1–V7 are retained to explain the design history. Their equations and historical interfaces were not silently adapted during archival publication.

## Plant evolution

| Stage | Main addition | Available evidence |
|---|---|---|
| V1 | Planar motion, four contact forces, kinematic/dynamic comparison; separate Fx/Fy limits | Code and embedded MLX outputs |
| V2 | Per-wheel telemetry | Code and embedded MLX outputs |
| V3 | Shared longitudinal/lateral adhesion circle | [Figures](../Results/V3_Friction_Circle/README.md) |
| V4 | Longitudinal/lateral load transfer and load-dependent stiffness | [Figures](../Results/V4_Load_Transfer/README.md) |
| V5 | Rear lateral-capacity loss/recovery under excess traction demand | [Figures and videos](../Results/V5_Slip_Tests/README.md) |
| V6 | Velocity projection into individual wheel coordinates | Code and embedded MLX outputs |
| V7 | Manoeuvre extraction and function organization | Historical code with older interface dependencies |
| V8 | Approximate braking-lock mode, recovery and complete state | [Braking tests](../Results/V8_Braking/README.md) |

See [Versions](../Documentation/Versions/README.md) for each stage's motivation and outcome.

## Plant interface

```matlab
[z_next, tele] = pasoDinamico(z, u, p, dt);
u = [requested_total_Fx; wheel_steering_delta];
```

`Fx` is **force [N]**, not power. It is positive for driving and negative for braking. `delta` is the front-wheel steering angle [rad], not a steering-wheel angle converted through a steering ratio. `p` stores vehicle, geometry, tyre, limit and time-constant parameters; `dt` is in seconds.

Wheel order is **DI, DD, TI, TD** in the source: front left, front right, rear left, rear right. Current drive allocation is `[0 0 .5 .5]`; braking allocation is `[.3 .3 .2 .2]`. Independent per-wheel braking commands are not yet provided.

| z indices | Meaning |
|---|---|
| 1:6 | X [m], Y [m], psi [rad], vx [m/s], vy [m/s], r [rad/s] |
| 7:8 | Previous-step physical accelerations used for load transfer |
| 9:12 | Per-wheel `eta_slip` memory |
| 13:16 | Per-wheel `eta_brake` memory |
| 17:20 | Approximate per-wheel lock mode, 0/1 |

There are six motion variables and fourteen memory components, not twenty mechanical degrees of freedom. Initialization from six motion components adds zero accelerations, unit adhesion factors and zero lock flags. Direct calls to `pasoDinamico` require the complete 20-component vector.

`tele` includes `Fxw`, `Fyw`, `Fzw`, `Fmax_w`, `Fmax_peak_w`, `lambda_w`, `alpha_w`, `Caw`, `eta_slip`, `eta_brake`, `bloqueo_w`, chassis-frame forces and accelerations. `Fxw/Fyw` use wheel coordinates: chassis-frame transformations are required for vehicle force balances. Step telemetry and next-state values must not be confused in time.

## Force and adhesion model

Vertical loads are redistributed quasi-statically using previous accelerations. Lateral stiffness scales with load:

```matlab
Caw = Caw0 .* (Fzw ./ Fzw0).^nCa;
lambda = min(1, Fmax ./ hypot(Fx_requested, Fy));
```

The common circle factor constrains combined force. An additional rear lateral-capacity memory depends on excess requested drive force.

During braking, demand relative to **nominal** capacity determines entry/exit from the lock mode, with hysteresis. Its reduced-capacity target is:

```matlab
eta_obj = brake_slide_ratio + ...
    (1-brake_slide_ratio)*exp(-abs(vx_local)/v_brake_slide);
mu_eff = mu_base * eta_brake;
```

`eta_brake` approaches this target exponentially, with separate time constants for loss, partial relief and full brake release. Test parameters are a 0.68 minimum ratio, 15 m/s speed scale, 0.03 s loss, 0.10 s partial recovery and 0.01 s full-release recovery. These are illustrative parameters awaiting calibration.

In the special lock mode, `Fy` is zero and longitudinal braking force is not rotated with steering; braking acts along chassis X. This prevents artificial steering through `Fx*sin(delta)`. A unit longitudinal factor preserves the already-reduced braking limit: **it does not mean Fx=1 N or removal of saturation**. This is a deliberately simplified test assumption.

## Running and compatibility

Set this directory as MATLAB's Current Folder and run the V8 `.mlx`. The six [manoeuvre scripts](Ensayos/README.md) target the current V8 interface. Do not assume V7 is interchangeable: its dynamic step still requires separate memory arguments, while the current straight-line test expects four inputs.

V1–V6 are self-contained historical files with differing parameters and signatures. They were not all rerun during publication, and current helpers are not guaranteed to regenerate every historical figure.

## Physical limitations retained

Wheel angular velocities, torque and rotational inertia are absent. Thus `bloqueo_w` does not establish mechanical wheel locking and `eta_slip` is not a physical slip ratio. Dynamic suspension, roll, pitch and experimentally calibrated tyre forces are not included. Vertical equilibrium and the friction circle are internal consistency checks, not experimental calibration. Braking is not universally limited to 1 g: adhesion, load and resistance matter.

The slip-angle convention needs future review: V6–V8 project velocities into the steered wheel frame and then calculate `delta-atan2(vy_local,vx_local)`. This may introduce steering twice. It is retained under the decision to freeze the plant and should be checked before claiming physical validity.


That historical slip-angle inconsistency was subsequently corrected in the active braking branch when Ackermann was added: the wheel-frame angle is `-atan2(vy_local,vx_local)`, without introducing steering twice. The original V6–V8 files remain unchanged for traceability. [Ackermann checks →](../Results/Ackermann/README.md)
