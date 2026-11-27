# Modulated ABS and working estimator configuration

## Decision and scope

On 27 November the user approved replacing the instantaneous force limiter with stateful wheel-by-wheel braking modulation, archiving the previous package, then comparing ABS off/on with **TC on in both**. The physical plant and TC remain unchanged. Sensors and EKF equations, sensor noise, R and P0 remain unchanged. The selected working Q is again `diag([.001 .001 .00001 .003 .003].^2)`, following the explicit decision to use the configuration that completes the course while awaiting academic review.

The earlier Q-restoration record is historical. A [restored-Q rerun](../../Results/Estimator_Q_Restored/README.md) fails at 6.046 s. The [four-case X/Y-only sweep](../../Results/Estimator_Q_XY_Study/README.md) also fails in every case. This does not establish a unique cause or a minimal successful Q. Q is process uncertainty, distinct from sensor-noise covariance R; no sensor errors were reduced for this update.

## Force command and state machine

For each wheel, retain the existing capacity calculation `Fmax = eta_brake * mu * Fz`, with the same load transfer. The output braking magnitude is bounded by the driver request and `factor * Fmax`. No lateral-force subtraction has been added to ABS. Channels use the established order `[DI DD TI TD]`.

| Parameter | Selected initial value | Role |
|---|---:|---|
| Activation factor | 0.98 | Starts modulation when requested braking reaches this fraction of current Fmax |
| Discharge factor X | 0.80 | Partial release target |
| Discharge duration | 30 ms | Linear 0.98→0.80, normalized slope −6/s |
| Hold duration | 10 ms | Keep the lower factor |
| Rebuild duration | 60 ms | Linear 0.80→0.98, normalized slope +3/s |

Persistent excess demand repeats the nominal 100 ms cycle (10 Hz). Each wheel has independent factor, phase and elapsed-time memory. Positive/zero commands pass through and reset that channel; traction or emergency resets the coordinator memory. Emergency retains priority and bypasses ABS/TC. Rear motor force remains equally split, with individual braking commands.

`controlABS` now returns `[FxCorregidas, info, mem]` from `(FxSolicitadas,z,p,mem,dt)`. Braking requires explicit carried memory; an old stateless braking call raises an explanatory error. Nonbraking legacy calls remain valid. `controlTotal` carries the memory and reads `p.abs.habilitado`; switching ABS off does not switch TC off. Changes in capacity or driver request can still change absolute output immediately: the ramp is on the normalized factor, not a hydraulic pressure actuator.

## Evidence and calibration limits

The public literature supports release/hold/rebuild architecture and millisecond actuation, but does **not** identify a universal X or force-ramp profile for Formula Student or Brembo racing ABS. These values are **project design assumptions**, not measured motorsport calibration.

- [Ferro, 2014, institutional record](https://scholar.tecnico.ulisboa.pt/records/UQtafsgSBvTm7n_HqCGMUaRj52gbTPsI3U6b?lang=en) and [simulation paper](https://repositorio.uisek.edu.ec/bitstream/123456789/2674/19/2014%20Dise%C3%B1o%20y%20simulaci%C3%B3n%20de%20un%20esquema%20de%20control%20de%20ABS.pdf): Formula Student FST05e cascade slip/pressure control, PWM at 50 Hz. PWM frequency is not a full ABS-cycle frequency, and pressure ratios cannot be treated directly as force/capacity ratios.
- [Bosch Motorsport M5 data sheet](https://www.bosch-motorsport.de/content/downloads/raceparts/Resources/pdf/Data%20Sheet_70612107_ABS_M5_Kit.pdf): per-wheel valves and wheel-speed/pressure/IMU signals with application-specific calibration; no public X or equivalent force ramps.
- [Bakirci and Katadzic, Chalmers, 2014](https://publications.lib.chalmers.se/records/fulltext/211567/211567.pdf), printed p.28/table 7: a secondary Bosch timing example motivates a 30 ms discharge scale, not a Formula Student or M5 calibration. The separate bump-mitigation timing table is not an ABS-cycle specification.
- [Bosch ABS overview](https://www.bosch-mobility.com/en/solutions/braking-systems/antilock-braking-system/) and [Brembo motorsport components](https://www.brembo.com/en/motorsport/powersport): general modulation/component context, no transferable X/force-ramp calibration.

X=.80 lies below the plant's existing .90 unlock threshold with margin. The 10 ms hold and 60 ms rebuild are initial choices. This remains a **force-capacity approximation**, with no new wheel-slip feedback or hydraulic plant and no racing validation. No ESC yaw controller has been added.

## Verification and outcomes

[Bench and circuit evidence](../../Results/Modulated_ABS/README.md) records phase timing, independent channels, reset behaviour, matching 1/2 ms discretizations and the full-loop comparison. With TC on in both, ABS off completes in 49.250 s with 2.016 m maximum path error; modulated ABS on completes in 47.294 s with 0.35427 m maximum error. Both use the same seed and noisy estimated loop. This supports the current simulation comparison, not general optimality or cone-clearance certification.

The previous complete package is [archived](../../Archive/2026-11-27_Before_Modulated_ABS/README.md). Historical videos retain their original controller and Q context.
