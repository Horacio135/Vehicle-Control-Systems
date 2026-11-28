# Four perfect-state cases

[Campaign overview](../README.md)

All cases share the circuit, profile, initial rest state and driver. Sensors/EKF are bypassed explicitly.

| Aids | End | Time | Progress |
|---|---|---:|---:|
| None | Sideslip exceeds 85 degrees | 11.722 s | 5.873% |
| ABS only | Sideslip exceeds 85 degrees | 11.806 s | 5.930% |
| TC only | Sideslip exceeds 85 degrees | 10.736 s | 6.029% |
| ABS + TC + ESC | Lap completed | 190.006 s | 100.000% |

The three early failures limit isolated-aid comparisons to the first sector. ABS avoids recorded wheel lock in its case but does not prevent lateral instability. TC alone does not stabilize the vehicle. These cases do not isolate every possible aid combination.

`Comparacion_completa.mat` contains the full four histories, configuration, track and profile; individual duplicate case MATs are omitted. `Resumen.csv`, the curve/sector inventories and PNGs complement the [x4 comparison video](Comparacion_4_coches_2D_x4.mp4). Failed cases are held at their last recorded state and labelled, never restarted or extrapolated. The reported path error concerns the centre, not an audited full-body clearance.
