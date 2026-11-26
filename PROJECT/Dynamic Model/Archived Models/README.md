# Archived post-V12 model alternatives

[← Historical plant](../README.md) · [Active package](../../Current%20State/README.md) · [Milestones](../../Documentation/Versions/After_V12/README.md)

These are source snapshots for reviewing equations, not competing default entry points. Do not recursively add these folders to the MATLAB path.

| Folder | Meaning |
|---|---|
| [Pacejka_V1](Pacejka_V1/) | Longitudinal wheel rotation in traction and braking |
| [Discarded_Pacejka_V2](Discarded_Pacejka_V2/) | Pure lateral and combined-slip extension, rejected configuration |
| [Traction_Only_Before_Ackermann](Traction_Only_Before_Ackermann/) | Rear traction MF plus restored historical braking |
| [Before_Ackermann](Before_Ackermann/) | Immediate physical snapshot before geometry correction |
| [Before_Sensor_Extension](Before_Sensor_Extension/) | Original GPS/longitudinal-IMU measurement interface |
| [Before_EKF_Extension](Before_EKF_Extension/) | Four-state P5 EKF before lateral motion was added |

Original Spanish identifiers/comments and source bytes are retained. The active runnable package supplies the current configuration and dependencies; archived alternatives are not silently adapted to its interfaces.
