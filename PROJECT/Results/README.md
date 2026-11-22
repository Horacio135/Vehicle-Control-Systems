# Simulation results

[← Project](../README.md) · [Videos](Hybrid_Tests/README.md) · [Numerical data](Numerical/README.md) · [Run sources](Run_Sources/README.md)

**Recorded evidence, including unsuccessful or degraded behaviour.** Existing videos were not rerendered and the models were not altered to obtain better trajectories.

## The 35 m/s comparison

Same fourteen-corner stress circuit:

| Configuration | Time [s] | Max. error [m] | RMS [m] | CG outside [%] | Lock mode [%] | Saturation [%] |
|---|---:|---:|---:|---:|---:|---:|
| V10 baseline | 51.384 | 1.263 | 0.497 | 0 | 12.276 | 45.429 |
| V11 demanding map | 47.680 | 0.776 | 0.238 | 0 | 19.869 | 78.629 |
| V12 extreme acceleration | 149.036 | 4.347 | 1.258 | 18.103 | 3.225 | 99.361 |

Percentages use time samples: lock mode if any wheel is flagged; saturation if any lambda<1-1e-6; outside if the CG crosses the track boundary. They do not describe the entire body envelope. V12's lower lock percentage does not demonstrate better braking: total duration changes substantially and traction loss dominates.

## Evidence by stage

| Stage | Available evidence |
|---|---|
| V1–V2 | Code and embedded MLX outputs in Dynamic Model; no independent metric tables |
| [V3](V3_Friction_Circle/README.md) | Five PNGs: friction circle, skidpad and telemetry |
| [V4](V4_Load_Transfer/README.md) | Nine PNGs: loads, stiffness and saturation |
| [V5](V5_Slip_Tests/README.md) | One figure and three drift/slalom videos |
| V6–V7 | Code and embedded MLX outputs; historical limitations documented |
| [V8](V8_Braking/README.md) | Braking, release and steering under approximate lock |
| [V9](V9_Speed_Planner/README.md) | Speed profiles, forces and tracking response |
| [V10](V10_Stress/README.md) | Stress circuit and abrupt driver demands |
| [V11](V11_Speed_Mapping/README.md) | More demanding mapping versus baseline |
| [V11 3D](V11_3D/README.md) | Single-car chase-camera scene |
| [V12](V12_Aggressive_Acceleration/README.md) | Extreme driving request and degraded tracking |
| [Hybrid videos](Hybrid_Tests/README.md) | Seven MP4s covering V8–V12 |
| [Numerical](Numerical/README.md) | Metrics for 50 histories and temporal CSV data |
| [Run_Sources](Run_Sources/README.md) | Source snapshots for historical configurations |

## What the results support

1. Speed mapping can improve lap time on this circuit without eliminating saturation or approximate locking.
2. Doubling requested drive force does not double transmitted force. V12 shows severe guidance loss and a much slower lap.
3. A 40 m/s cap does not mean the car reaches 40 m/s: V11 records a peak of 37.405 m/s.
4. A 3D display does not validate vertical or wheel-rotation dynamics that are absent.
5. These are numerical findings from a phenomenological model, not experimental validation or proof of an implemented ESC/ABS/TCS.

Current files may differ from the configurations that generated earlier videos. Filenames, configuration JSONs and source snapshots distinguish them. Not every early version has independent numerical records; its stage README states that explicitly.

Original plots and videos retain their recorded labels and filenames. The surrounding documentation, captions and metric definitions are in English.
