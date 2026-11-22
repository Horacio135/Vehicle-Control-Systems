# V10 · Stress circuit and abrupt driver demands

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Expose plant limits with more demanding corners and rapid throttle/brake application.

## Implementation

The 631.56 m circuit has fourteen corners, a chicane, a 90° R4 m corner after a 172 m straight, and a 70° R3.5 m corner. Maximum targets use exponential application: 95% in 90 ms for acceleration and 60 ms for braking.

**Source:** [Prueba_hibrida_V10_Estres_14_curvas.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V10_Estres_14_curvas.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

Campaigns at 6/10/14/18/22 and 15/20/25/30/35 m/s are retained. At 35 m/s: 51.384 s, 1.263 m maximum error, 12.276% approximate lock time, 0% CG outside. All five cases in both campaigns complete a lap.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Completion does not establish universal stability. Utilization and locking belong to the simplified model; no ESC modulates demand.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
