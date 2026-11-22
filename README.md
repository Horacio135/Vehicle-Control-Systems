# Vehicle-Control-Systems
This repository will contain projects regarding the course Vehicle Control Systems in which we will develop a controller for a Formula Student Driverless model in Matlab, focusing on improving and deep-diving in one aspect of the control system.

## Repository structure

The original course folders are retained. Development of the four-wheel vehicle and the future ESC is documented under **PROJECT**.

| Folder | Contents |
|---|---|
| [Base_Model](Base_Model/README.md) | Longitudinal vehicle model coursework |
| [Bicicle_Dynamic_Model](Bicicle_Dynamic_Model/README.md) | Bicycle model coursework; original folder name retained |
| [ACC controller design](ACC%20controller%20design/README.md) | Adaptive cruise control coursework |
| [Trajectory Tracking](Trajectory%20Tracking/README.md) | Pure Pursuit coursework used as a project reference |
| [PROJECT](PROJECT/README.md) | Four-wheel model V1–V8, trajectory tracking, speed planning and stress tests through V12 |

## Project status — 22 November 2026

Implemented: per-wheel forces, combined adhesion saturation, quasi-static load transfer, phenomenological loss/recovery of lateral and braking capacity, Pure Pursuit, curvature-based speed planning, aggressive-driver scenarios, and 2D/3D animations.

**ESC, ABS and traction control are future work.** The simulations already include speed and path controllers; “without stability control” does not mean an entirely open-loop vehicle. Wheel angular speeds, wheel inertias and physical longitudinal slip are not modelled. A 3D display visualizes planar dynamics.

The project documentation is in Spanish to preserve the design discussion:

- [Start and run the project](PROJECT/README.md)
- [Design decisions and V1–V12 history](PROJECT/Documentation/README.md)
- [Results, numerical evidence and videos](PROJECT/Results/README.md)
- [Current plant interface](PROJECT/Dynamic%20Model/README.md)

The latest aggressive test completes a lap in **149.036 s**, versus **47.680 s** for its V11 baseline, and reaches **4.347 m** lateral error. More requested force does not imply better tracking. These are simulation findings, not experimental vehicle validation.
