# Modular integration and tyre experiments

[← Results](../README.md) · [Current full loop](../Closed_Loop/README.md) · [Design rationale](../../Documentation/Dynamics_Development.md)

| Stage | What it records |
|---|---|
| [Initial](Initial/README.md) | Early ABS/TC integration and realtime 2D/3D renderings |
| [Pure Pursuit](PurePursuit_Adjustment/README.md) | Tracking refinement and contemporary simulations |
| [Discarded longitudinal eta](Discarded_Longitudinal_Eta/README.md) | Rejected loss-factor experiment |
| [Traction characterization](Traction_Characterization/README.md) | Illustrative slip curves |
| [Pacejka V1](Pacejka_V1/README.md) | Longitudinal rotational model, including braking |
| [Discarded V2](Discarded_Pacejka_V2/README.md) | Lateral/combined extension that fails the circuit |
| [Traction-only](Traction_Only/README.md) | Selected hybrid before Ackermann |

The [earlier realtime 3D video](Initial/ABS_TC_realtime_3D.mp4) and [2D video](Initial/ABS_TC_realtime_2D.mp4) are historical integration evidence. They do not use the current seven-state EKF. Later 3D work has not been rendered for the current complete loop.

Identical media are stored once. The [artifact manifest](../../Documentation/closed_loop_artifact_manifest.json) maps duplicate originals to their published file. All numerical findings are simulation-specific and lack experimental tyre identification.
