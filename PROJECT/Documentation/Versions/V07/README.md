# V07 · Separate manoeuvre scripts

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Separate manoeuvres and presentation from the core in preparation for modular architecture.

## Implementation

External helpers are placed in Ensayos; previous-step memories still enter the dynamic step as additional arguments.

**Source:** [CuatroruedasV7_estructurado.mlx](../../../Dynamic%20Model/CuatroruedasV7_estructurado.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

The file records the pre-normalization interface. Current helpers target V8 and do not provide a complete historical V7 environment.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Its dynamic step requires ax_prev, ay_prev and eta_prev; the current straight-line test expects only z,u,p,dt. Full execution against current dependencies is not promised.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
