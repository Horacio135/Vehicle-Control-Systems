# X/Y process-covariance study — 27 November 2026

Four versions use `Q = diag([s s .005 .10 .10].^2)`, with `s = .020, .015, .010, .005`. The 0.005 step applies to the values **before squaring**, not directly to covariance entries. Only the first two values vary; all other Q entries, sensor noise, R, P0, controller equations, track, speed plan and seed remain fixed. ABS/TC are enabled, using the previous instantaneous ABS.

| s | Qxx = Qyy | Outcome | Simulated time [s] |
|---|---:|---|---:|
| .020 | .000400 | Path error >10 m | 6.046 |
| .015 | .000225 | Path error >10 m | 5.944 |
| .010 | .000100 | Path error >10 m | 5.994 |
| .005 | .000025 | Path error >10 m | 12.670 |

None completes the lap. These are failure times, not lap times, and the sweep does not identify a minimally modified successful Q. The later choice returns to `diag([.001 .001 .00001 .003 .003].^2)`, which also changes yaw and velocity uncertainties; success cannot be attributed solely to X/Y entries.

[CSV](resumen_versiones.csv) · [Combined histories](estudio_completo.mat) · [Configuration verification](verificacion_configuraciones.json)

Each version directory contains its MAT history and a normal-speed 2D actual/estimated video:

- [s=.020](Version_XY_0_020/Circuito_Q_XY_ABS_TC_real_estimado_2D.mp4)
- [s=.015](Version_XY_0_015/Circuito_Q_XY_ABS_TC_real_estimado_2D.mp4)
- [s=.010](Version_XY_0_010/Circuito_Q_XY_ABS_TC_real_estimado_2D.mp4)
- [s=.005](Version_XY_0_005/Circuito_Q_XY_ABS_TC_real_estimado_2D.mp4)

`Renderizar_estudio_Q_XY.m` replays the stored cases without rerunning the physical blocks. No failed sweep version is promoted to current state.
