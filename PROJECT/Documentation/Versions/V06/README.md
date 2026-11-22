# V06 · Individual wheel-frame calculations

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Express contact velocities and forces in each wheel's oriented frame.

## Implementation

vxw and vyw are projected into wheel coordinates while retaining per-wheel forces, saturation, loads and lateral-capacity loss.

**Source:** [CuatroruedasV6_dinamica_indiv_rueda.mlx](../../../Dynamic%20Model/CuatroruedasV6_dinamica_indiv_rueda.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

Code and embedded MLX outputs are retained. There is no independent V6 metrics dataset; V5/V8 values are not reassigned to it.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

The delta-atan2(vy_local,vx_local) convention needs future review for possible double introduction of delta. Publication does not alter the physics.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
