# V01 · Initial dynamic model

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Compare kinematic and dynamic responses while retaining a planar chassis.

## Implementation

Per-contact forces, chassis transformations and yaw moments are calculated alongside aerodynamic and rolling resistance. Fx and Fy still have independent limits.

**Source:** [CuatroruedasV1_simulacion.mlx](../../../Dynamic%20Model/CuatroruedasV1_simulacion.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

The code and embedded MLX outputs are the retained evidence. No independent V1 numerical campaign is available; no error measurements are invented.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Independent force limits do not correctly share adhesion under combined demands. This motivates V3.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
