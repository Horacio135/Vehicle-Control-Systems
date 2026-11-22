# V11 · Demanding speed mapping and 3D visualization

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Reduce corner-entry speed less and start braking later without changing the plant.

## Implementation

ayMax=11, axFreno=12 m/s² and anticipacionPI=0; axAcel remains 6. A V10 baseline comparison and a single-car 35 m/s 3D variant are retained.

**Source:** [Prueba_hibrida_V11_Mapeo_limite.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V11_Mapeo_limite.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

At 35 m/s: 47.680 s, 0.776 m maximum error, 19.869% lock time and 0% CG outside. Planned braking starts 25.5 m later on the initial straight. Current caps are 25/30/32/35/40 m/s; the 40 case reaches only 37.405 m/s peak vx.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

The improvement is scenario-specific and increases approximate locking. The 3D display retains planar physics and does not add wheel rotation.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
