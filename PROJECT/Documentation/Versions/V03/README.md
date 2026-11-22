# V03 · Friction circle

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Prevent Fx and Fy from independently consuming the entire same contact capacity.

## Implementation

A common factor limits resultant force. Telemetry shows adhesion utilization and saturation during turning.

**Source:** [CuatroruedasV3_simulacion_circulo_adherencia.mlx](../../../Dynamic%20Model/CuatroruedasV3_simulacion_circulo_adherencia.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

Five figures are retained: skidpad and telemetry at 5/30 m/s, plus friction-circle utilization. Their labels describe those runs, not the single setting in the current MLX.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Saturation alone models neither mechanical wheel lock nor a real tyre's post-peak force drop.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
