# V04 · Load transfer

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Distribute contact capacity differently during acceleration, braking and cornering.

## Implementation

Longitudinal/lateral transfer uses previous accelerations, and lateral stiffness depends on Fz. Slalom and standstill scenarios are expanded.

**Source:** [CuatroruedasV4_simulacion_transferencia_pesos.mlx](../../../Dynamic%20Model/CuatroruedasV4_simulacion_transferencia_pesos.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

Nine figures retain wheel loads, stiffness, utilization, skidpad and slalom. Tests are labelled at 5 and 14 m/s. Vertical equilibrium and load response are internal consistency checks.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Load transfer is quasi-static: dynamic suspension, roll and pitch are not introduced.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
