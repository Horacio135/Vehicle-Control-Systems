# V12 · Extreme acceleration demand

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Request more driving force more rapidly even if corner-exit tracking deteriorates.

## Implementation

Maximum driving-force request rises from 2500 to 5000 N and tauAcel drops from 0.030 to 0.010 s, reaching 95% in about 30 ms. Braking, mapping, circuit, tracking and plant equations are unchanged.

**Source:** [Prueba_hibrida_V12_35ms_3D_Aceleracion_agresiva.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V12_35ms_3D_Aceleracion_agresiva.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

Lap time is 149.036 s; maximum error 4.347 m; RMS 1.258 m; CG outside 18.103%. Minimum rear lateral factor drops from 0.153 to 0.078. Rear longitudinal excess during acceleration rises from 49.664% to 98.556%. The complete MP4 lasts 149.133 s at 15 fps.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

This is not a performance improvement. It is an extreme adhesion-loss scenario for future stability-control evaluation. Track exits are neither hidden nor corrected through physical-model changes.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
