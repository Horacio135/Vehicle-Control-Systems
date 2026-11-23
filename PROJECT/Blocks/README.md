# Callable project blocks

[← Project](../README.md) · [Plant-only simulation template](../Modular%20Architecture/README.md)

Five new callable blocks are exposed alongside the unchanged dynamic plant and its initializer. Longitudinal/lateral control comes from V12; sensing, the EKF and TTC emergency logic come from solved practice 5. This step separates the functions; it does not yet connect the complete closed-loop architecture or implement ESC, TC or ABS.

## Inputs and outputs

| MATLAB file | Call | Main output |
|---|---|---|
| [controlEmergencia.m](controlEmergencia.m) | `[FxE,deltaE,active,memE]=controlEmergencia(est,obst,delta,p,memE)` | Emergency braking demand and steering to retain |
| [controlLateral.m](controlLateral.m) | `[delta,memA]=controlLateral(est,path,p,memA)` | Requested front-wheel angle [rad] |
| [controlLongitudinal.m](controlLongitudinal.m) | `[Fx,memL]=controlLongitudinal(est,ref,p,memL,dt)` | Requested total longitudinal force [N] |
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

This illustrates command arbitration, not a complete sensing/estimation simulation loop. ESC/TC/ABS corrections are still absent.

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
