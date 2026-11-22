# Vehicle Control Systems

![Vehicle Control Systems: four-wheel dynamics, trajectory tracking and stability-control development](PROJECT/Documentation/assets/project_overview.svg)

![MATLAB R2026a](https://img.shields.io/badge/MATLAB-R2026a-0076A8)
![Project stage: V12](https://img.shields.io/badge/Project-V12-148F77)
![ESC: planned](https://img.shields.io/badge/ESC-planned-D99A21)

**A Formula Student Driverless vehicle model, from four-wheel dynamics to trajectory tracking and demanding stability tests.**

MATLAB coursework and project development for Vehicle Control Systems. The repository records how the physical model evolved, why design decisions were made, and what each simulation actually demonstrated.

[Quick start](#quick-start) · [Architecture](PROJECT/README.md#architecture) · [V1–V12 history](PROJECT/Documentation/Versions/README.md) · [Results](PROJECT/Results/README.md) · [Videos](PROJECT/Results/Hybrid_Tests/README.md)

## At a glance

| Implemented | Next development phase |
|---|---|
| Four-wheel forces, combined adhesion and quasi-static load transfer | Assemble the control architecture around the frozen plant |
| Phenomenological loss and recovery of braking/lateral capacity | Independent wheel-force allocation |
| Pure Pursuit and curvature-based speed planning | Coordinated ESC, ABS and traction control |
| Aggressive-driver tests and 2D/3D animations | Physical wheel-slip modelling and experimental calibration |

**The speed and path controllers are implemented; ESC, ABS and TCS are not.** Wheel angular velocities and rotational inertia are not simulated. The 3D scene displays planar dynamics.

## Quick start

```bash
git clone https://github.com/Horacio135/Vehicle-Control-Systems.git
cd Vehicle-Control-Systems
```

In MATLAB, open the repository root and run:

```matlab
cd(fullfile('PROJECT','Hybrid Tests'))
open('Prueba_hibrida_V11_35ms_3D.mlx')
```

Run the entire Live Script for the single-car 35 m/s scenario. To explore the extreme acceleration case, open `Prueba_hibrida_V12_35ms_3D_Aceleracion_agresiva.mlx` instead. MATLAB **R2026a** was used for verification; the native 3D graphics require no additional visualization library. Earlier releases have not been certified.

## Key result: more requested force can mean a worse lap

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

- [Plant interface and physical assumptions](PROJECT/Dynamic%20Model/README.md)
- [Running and configuring the hybrid tests](PROJECT/Hybrid%20Tests/README.md)
- [Twelve development stages](PROJECT/Documentation/Versions/README.md)
- [Design decisions and remaining limitations](PROJECT/Documentation/Design_Decisions/README.md)
- [50 recorded histories and telemetry](PROJECT/Results/Numerical/README.md)
- [Reproducing a recorded run](PROJECT/Tools/README.md)

Documentation is maintained in English. Historical filenames, MATLAB identifiers and original recorded graphics retain their established names so that existing scripts, links and result provenance remain traceable.
