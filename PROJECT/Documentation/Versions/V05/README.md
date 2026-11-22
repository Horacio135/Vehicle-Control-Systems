# V05 · Lateral loss under excess traction demand

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Show how excessive driving demand degrades rear-axle guidance capacity.

## Implementation

eta_slip provides per-wheel memory and additional rear lateral loss when demand exceeds capacity. Recovery is progressive.

**Source:** [CuatroruedasV5_simulacion_deslizamiento_longitudinal.mlx](../../../Dynamic%20Model/CuatroruedasV5_simulacion_deslizamiento_longitudinal.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

V5_Slip_Tests retains one figure and three drift/skidpad/slalom videos. They are visual evidence of the approximate model.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

The historical filename does not imply physical slip ratio: wheel omega and rotational inertia are absent.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
