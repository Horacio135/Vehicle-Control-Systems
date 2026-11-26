# Vehicle Control Systems

![Vehicle Control Systems: four-wheel dynamics, trajectory tracking and stability-control development](PROJECT/Documentation/assets/project_overview.svg)

![MATLAB R2026a](https://img.shields.io/badge/MATLAB-R2026a-0076A8)
![Project stage: closed loop](https://img.shields.io/badge/Project-closed_loop-148F77)
![ESC: planned](https://img.shields.io/badge/ESC-planned-D99A21)

**A Formula Student Driverless vehicle model with a complete sensing, estimation and force-control simulation loop.**

MATLAB coursework and project development for Vehicle Control Systems. The repository records how the physical model evolved, why design decisions were made, and what each simulation actually demonstrated.

[Watch simulation videos](https://Horacio135.github.io/Vehicle-Control-Systems/) · [Quick start](#quick-start) · [Current package](PROJECT/Current%20State/README.md) · [Theory](PROJECT/Documentation/Dynamics_Development.md) · [History](PROJECT/Documentation/Versions/README.md) · [Current videos](PROJECT/Results/Closed_Loop/README.md)

**Latest update — 26 November 2026:** the active estimator configuration restores `Q = diag([.02 .02 .005 .10 .10].^2)`. Sensor-noise parameters, R, estimator equations and P0 are unchanged. The 47.370 s lap and associated full-loop videos below were recorded with the previous reduced Q; no complete-loop rerun has validated restored Q. The latest retained steering evidence is [moose-style test 02](PROJECT/Results/Moose_Test_02/README.md), a failed Pure Pursuit manoeuvre without sensors or estimation. [Decision record](PROJECT/Documentation/Design_Decisions/Q_Restoration_and_Moose_Test_2026-11-26.md).

## At a glance

| Implemented | Next development phase |
|---|---|
| Four-wheel plant, load transfer and ideal Ackermann steering | Experimental identification and operating-range validation |
| Rear traction-only Pacejka with wheel rotation; historical braking | Review extreme-motion plant/estimator consistency |
| GPS/IMU/steering measurements and seven-state EKF | Bias estimation, covariance consistency and robustness tests |
| Pure Pursuit, speed planning and force-based ABS/TC | ESC yaw control and combined-capacity planning |
| Current 2D full-loop videos and historical 2D/3D evidence | A current full-loop 3D rendering |

**The complete loop is connected.** Its saved reduced-Q controlled run completes the circuit in **47.370 s** with **0.350 m** maximum path error. ABS/TC remain force limiters, not physical-slip feedback regulators, and no ESC yaw controller has been added. Rear wheel rotation is dynamic during traction; braking retains the earlier approximate model. All 3D scenes still display planar physics.

## Quick start

```bash
git clone https://github.com/Horacio135/Vehicle-Control-Systems.git
cd Vehicle-Control-Systems
```

In MATLAB, open the repository root and run:

```matlab
cd(fullfile('PROJECT','Current State'))
open('Integracion_estado_actual.mlx')
```

Run the entire Live Script. It compares estimated-state cases with ABS/TC off and on, plus an explicitly separate perfect-state reference. New local results go to the package's `Resultados` folder. MATLAB **R2026a** was used for verification; earlier releases have not been certified. Historical V11/V12 3D scenarios remain available in [Hybrid Tests](PROJECT/Hybrid%20Tests/README.md), but do not represent the current estimated loop.

## Saved full-loop results with previous reduced Q

| Case | Lap time [s] | Maximum path error [m] | Position-estimator RMSE [m] |
|---|---:|---:|---:|
| Estimated loop, ABS/TC on | 47.370 | 0.350 | 0.142 |
| Estimated loop, ABS/TC off | 101.228 | 7.258 | 3.145 |
| Perfect-state reference, ABS/TC on | 47.132 | 0.329 | — |

The uncontrolled case completes the route but leaves the corridor. A failed first attempt, the provisional Q adjustment and same-measurement estimator replays are retained in [the results](PROJECT/Results/Closed_Loop/README.md) and [failure analysis](PROJECT/Documentation/Estimator_Q_and_Failure_Analysis.md). A successful lap is not experimental validation or proof of robustness during severe spins.

## Historical result: more requested force can mean a worse lap

Same 631.56 m circuit, same 35 m/s speed cap:

| Configuration | Lap time [s] | Maximum lateral error [m] | CG outside track [%] |
|---|---:|---:|---:|
| V10 — baseline speed map | 51.384 | 1.263 | 0 |
| V11 — more demanding speed map | 47.680 | 0.776 | 0 |
| V12 — extreme acceleration request | 149.036 | 4.347 | 18.103 |

V12 doubles the maximum requested driving force and applies it faster. Adhesion saturation and rear lateral-capacity loss make tracking substantially worse. These are **simulation findings**, not experimental vehicle validation. [Read the comparison and measurement definitions →](PROJECT/Results/README.md)

## Repository map

The original coursework/project separation is retained.

| Directory | Purpose |
|---|---|
| [Base_Model](Base_Model/README.md) | Longitudinal vehicle dynamics and PI speed control |
| [Bicicle_Dynamic_Model](Bicicle_Dynamic_Model/README.md) | Bicycle-model coursework; original directory name retained |
| [ACC controller design](ACC%20controller%20design/README.md) | Adaptive cruise control coursework |
| [Trajectory Tracking](Trajectory%20Tracking/README.md) | Pure Pursuit coursework used as the project reference |
| [PROJECT](PROJECT/README.md) | Four-wheel plant, hybrid tests, design history and evidence |

## Explore the project

- [Current complete-loop package](PROJECT/Current%20State/README.md)
- [Dynamics theory and post-V12 decisions](PROJECT/Documentation/Dynamics_Development.md)
- [Estimator Q and straight-line departure analysis](PROJECT/Documentation/Estimator_Q_and_Failure_Analysis.md)
- [Historical plant interface and physical assumptions](PROJECT/Dynamic%20Model/README.md)
- [Running and configuring the hybrid tests](PROJECT/Hybrid%20Tests/README.md)
- [Twelve development stages](PROJECT/Documentation/Versions/README.md)
- [Design decisions and remaining limitations](PROJECT/Documentation/Design_Decisions/README.md)
- [50 recorded histories and telemetry](PROJECT/Results/Numerical/README.md)
- [Reproducing a recorded run](PROJECT/Tools/README.md)
- [Historical modular extraction and plant-only template](PROJECT/Modular%20Architecture/README.md)

Documentation is maintained in English. Current MATLAB identifiers and comments remain in Spanish as requested; historical names and recorded graphics retain their provenance. The published current source mirrors the verified local package without changing its block equations.
