# V08 · Approximate lock, recovery and plant interface

[← All stages](../README.md) · [Design decisions](../../Design_Decisions/README.md) · [Results](../../../Results/README.md)

## Problem and decision

Distinguish normal saturation from capacity loss under excessive braking and prepare controller integration.

## Implementation

A hysteretic mode adds speed-dependent capacity, rapid release recovery and zero lateral force during locking. Memories move into the 20-component z, leaving four inputs and two outputs.

**Source:** [CuatroruedasV8_bloqueo_ruedas.mlx](../../../Dynamic%20Model/CuatroruedasV8_bloqueo_ruedas.mlx). Readable `.m` code is available in that family's Source Exports directory.

## Simulations and results

Evidence includes stopping after acceleration, a 50 ms brake release, two turns after 4 m of braking, and before/after state-interface comparisons. The straight-line record reaches 35.002 m/s and ends at 7.472 s including acceleration and braking; this is not braking time alone.

Artifacts are indexed in [Results](../../../Results/README.md). Recent metrics come from [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculated over full-resolution records; exported temporal CSVs retain samples at approximately 20 Hz.

## Interpretation and limitations

The mode does not calculate physical lock. ABS/TCS and independent wheel actuation are absent. Hybrid V8 adds Pure Pursuit without changing the plant.

Each stage's evidence retains its own configuration. The entire history has not been rerun, and measurements are not transferred between versions.
