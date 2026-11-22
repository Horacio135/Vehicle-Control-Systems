# V02 · Four-wheel telemetry

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Observe each wheel's forces and relate them to the vehicle trajectory.

## Implementation

pasoDinamico returns telemetry as well as the state. Logging and circular-manoeuvre plots are expanded.

**Source:** [CuatroruedasV2_simulacion_4ruedas.mlx](../../../Dynamic%20Model/CuatroruedasV2_simulacion_4ruedas.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

The MLX and readable code export are retained. The file configures an 18 m/s skidpad; this value must not be attributed to every historical figure.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Longitudinal and lateral forces still need a shared capacity constraint. Wheel angular velocities are not calculated.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
