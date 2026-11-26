# Historical callable-block extraction

[← Project](../README.md) · [Plant-only simulation template](../Modular%20Architecture/README.md)

**This page describes the earlier extraction stage, not the active complete loop.** Use [Current State](../Current%20State/README.md) for the seven-state EKF, Ackermann hybrid plant, connected force-based ABS/TC and external adapters. [Current evidence](../Results/Closed_Loop/README.md) includes the successful estimated loop and the retained failed attempt.

The historical stage exposed callable blocks alongside its unchanged plant and initializer. Longitudinal/lateral control came from V12; sensing, the four-state EKF and TTC emergency logic came from practice 5. Its original contracts and limitations are retained below. Statements that integration or ABS were pending refer to this extraction stage. No ESC yaw law has been added in the current package either.

## Inputs and outputs

| MATLAB file | Call | Main output |
|---|---|---|
| [controlEmergencia.m](controlEmergencia.m) | `[FxE,deltaE,active,memE]=controlEmergencia(est,obst,delta,p,memE)` | Emergency braking demand and steering to retain |
| [controlLateral.m](controlLateral.m) | `[delta,memA]=controlLateral(est,path,p,memA)` | Requested front-wheel angle [rad] |
| [controlLongitudinal.m](controlLongitudinal.m) | `[Fx,memL]=controlLongitudinal(est,ref,p,memL,dt)` | Requested total longitudinal force [N] |
| [controlTraccion.m](controlTraccion.m) | `[FxRear,info]=controlTraccion(FxRearRequested,z,p,dt)` | Corrected rear-wheel traction requests [N] |
| [estimadorEstado.m](estimadorEstado.m) | `[est,memK]=estimadorEstado(med,uPrevious,p,memK,dt)` | P5 estimated state |
| [sensores.m](sensores.m) | `med=sensores(z,tele,p,k)` | Noisy GPS/IMU measurements |
| [pasoDinamico.m](pasoDinamico.m) | `[zNext,tele]=pasoDinamico(z,u,p,dt)` | Complete next physical state and telemetry |
| [inicializarEstadoDinamico.m](inicializarEstadoDinamico.m) | `z=inicializarEstadoDinamico(motionState)` | Complete 20-component state |

References, physical/controller settings and persistent memories are explicit arguments: estimated state alone cannot define a desired path, target speed or obstacle location. Keep separate memories for lateral control, longitudinal control, the EKF and emergency logic.

## Emergency semantics

The P5 law checks whether a static obstacle is ahead and uses range / forward speed as TTC. Activation is latched until the caller resets `memE`. When active, the force request is `-p.Fx_brake_max`. When inactive, the emergency force output is zero and must **not** overwrite nominal driving demand.

P5 contains no evasive steering law. This block therefore receives `delta` from nominal lateral control and returns the same angle as `deltaE`, preserving P5 behaviour instead of forcing straight steering or inventing an avoidance manoeuvre.

```matlab
[delta,memA] = controlLateral(est,path,p,memA);
[Fx,memL] = controlLongitudinal(est,ref,p,memL,dt);
[FxE,deltaE,active,memE] = controlEmergencia(est,obst,delta,p,memE);
if active
    u = [FxE;deltaE];
else
    u = [Fx;delta];
end
```

This illustrates command arbitration, not a complete sensing/estimation simulation loop. It does not invoke the new traction limiter; ESC and ABS remain absent.

## Rear traction-demand limiter

```matlab
% Rear order: [TI TD] = [rear left, rear right].
p.tc_factor = 0.98;  % Optional override; the function defaults to 0.98.
[FxRearCorrected,info] = controlTraccion([FxTI FxTD],z,p,dt);
```

The block recomputes the same wheel-load transfer as the plant using the previous physical accelerations in z. It then uses `Fmax_peak_w = p.muw.*Fzw` and `Fmax_w = eta_brake.*Fmax_peak_w`. For traction, braking-capacity recovery over the current dt matches the plant's full-release update. The prediction is not written back to z: the physical block still updates its memory exactly once.

For each positive rear request, the rule is:

```matlab
FxCorrected = min(FxRequested, p.tc_factor * FmaxAvailable);
```

Zero and negative requests pass unchanged. A request containing only braking/zero values bypasses capacity calculation; its inactive diagnostics contain NaN capacities. Mixed-sign inputs limit only the positive component. A row or column input keeps its orientation. `info` exposes rear loads, nominal/effective capacities, thresholds and limitation flags.

The factor can be increased above one to deliberately permit demand beyond the calculated capacity. That can expose the existing loss-of-adhesion behaviour, but does not prescribe a particular oversteer angle or guarantee a faster lap. Zero suppresses positive demand entirely.

**Capacity definition:** the requested rule uses the plant's **combined-force radius**, not `sqrt(Fmax^2-Fy^2)`. Therefore 0.98 does not reserve a separately calculated lateral-force budget or guarantee no combined saturation in a corner.

**Historical integration limits:** z is the complete physical state, so this first block is a model-based/ideal-information limiter. The P5 estimator alone does not supply all its required capacity information. The output is independently limited per rear wheel, but the frozen plant still accepts a total force with fixed allocation. The limiter is not automatically connected, and summing unequal corrected requests does not reproduce their individual limits after the plant redistributes that total.

[Traction-limiter checks](traction_control_checks.json) verify exact capacity agreement with the plant, transfer asymmetry, post-brake recovery, threshold behaviour and unchanged braking requests.

## Measurements and estimates

`sensores` reads X, Y and yaw rate from `z`, and longitudinal physical acceleration from `tele.ax`. It applies P5 Gaussian noise, gyro bias, intermittent GPS and configurable GPS outage. The signal packet includes `acc`, `gyro`, `hay_gps`, `gps_X`, `gps_Y` and timestamp `t`. It does not pass perfect tyre forces or lock flags to the controllers as fictitious measurements.

Unlike the original P5 helper, this sensor block **does not recalculate bicycle acceleration**. Its acceleration truth comes from the actual four-wheel plant. Supply the state and telemetry corresponding to the measurement sample. Scheduling/caching those signals in the future complete loop remains a separate integration step.

The unchanged P5 EKF estimates only `[X,Y,psi,vx]`. It sets `est.vy=0` and uses `est.r=med.gyro`; it neither estimates lateral velocity nor removes gyro bias. Its acceleration integration is a simplified longitudinal approximation during lateral motion. `uPrevious` is retained for interface compatibility, although this original IMU-driven predictor does not use it.

## Parameters and initialization

The existing plant-only `runSimulation.m` now exposes these settings as well as the original physical parameters, while continuing to execute only the plant scenario:

```matlab
p.dt = cfg.dt;
p.Kp_v = 800; p.Ki_v = 200;
p.Ld0 = 2; p.kLd = 0.3; p.delta_max = 0.5;
p.conductor.tauAcel = 0.010; p.conductor.tauFreno = 0.020;
p.conductor.errorActivar = 0.35; p.conductor.errorSoltar = 0.05;
p.gps_sigma = 0.5; p.acc_sigma = 0.2; p.gyro_sigma = 0.01;
p.gps_cada = 25; p.gps_corte = []; p.gyro_bias = 0;
p.Q = diag([0.02 0.02 0.005 0.10].^2);
p.R = diag([p.gps_sigma p.gps_sigma].^2);
p.ttc_umbral = 2.0;
```

The drive/brake force limits still come from the scenario's physical `p` settings. The V12 acceleration time constant does not silently change `p.Fx_drive_max` from the plant-template value to 5000 N; set that request limit explicitly when selecting an aggressive scenario.

```matlab
memA = struct('idx',1);
memL = struct('Fx',0,'modo',0,'e_int',0);
memE = struct('latched',false);
memK.xh = [X0;Y0;psi0;vx0];  % Initial estimate, not the 20-component plant state
memK.P = diag([1 1 0.2 1]);
ref.v_set = targetSpeed;
```

Set `path` as an Nx2 reference trajectory, `obst.X/Y` as the static obstacle position, and preserve every returned memory. GPS sampling uses `p.dt`; the estimator and plant must use the same integration step. Set the random seed outside the sensors for repeatable tests.

## Frozen plant and relocation

The dynamic function and initializer have been moved here from Modular Architecture, byte-for-byte. The existing simulation template now points here. The plant still accepts `[Fx_total;delta]` and fixed force allocations; independent wheel braking remains a future interface decision.

[Plant provenance](plant_provenance.json) · [Original extraction checks](extraction_checks.json) · [Block source provenance](blocks_provenance.json)

[MATLAB block checks](block_checks.json) confirm exact V12 longitudinal/lateral outputs, exact solved-P5 EKF outputs, P5 emergency latching with steering preserved, GPS outages, configured noise statistics and unchanged plant-only results after relocation.
