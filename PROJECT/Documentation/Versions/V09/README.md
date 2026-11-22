# V09 · Curvature-based speed planner

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Accelerate on straights and slow before corners instead of following the first hybrid's constant speed reference.

## Implementation

Curvature limits, v² sweeps, periodic profile queries and a longitudinal PI are combined. Campaigns use 14/18/22/26, then 6/10/14/18/22 m/s.

**Source:** [Prueba_hibrida_V9_PurePursuit_animada.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V9_PurePursuit_animada.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

All five animated cases complete laps in 44.164, 31.628, 30.436, 30.432 and 30.432 s. Raising the cap above 14 m/s adds little here because curvature constrains the profile.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

Current non-animated V9 has six cases but 2×2 plot layouts; the limitation is documented. Animated five-case V9 is the recommended entry point.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
