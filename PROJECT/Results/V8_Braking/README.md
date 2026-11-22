# V8 · Braking and recovery

[← Results](../README.md) · [Metrics and telemetry](../Numerical/README.md)

Images record capacity evolution, a 50 ms full brake release and 2°/10° steering requests beginning 4 m after braking. Complete lock mode removes lateral guidance in this model. Rapid recovery is a property of its exponential memory, not a measured mechanical wheel response.

| Case | Speed cap [m/s] | Time [s] | Lap/stop reached | Max. error [m] | CG outside [%] | Lock mode [%] |
|---|---:|---:|---|---:|---:|---:|
| sharp turn | — | 7.470 | Yes | — | — | 36.590 |
| mild turn | — | 7.472 | Yes | — | — | 36.607 |

## Recorded artifacts

- [Straight-line braking capacity](capacidad_recta.png)
- [Sharp steering request during braking](giro_acusado.png)
- [Mild steering request during braking](giro_leve.png)
- [Full brake release for 50 ms](liberacion_50ms.png)
- [V8 straight-line test](recta_v8.png)
