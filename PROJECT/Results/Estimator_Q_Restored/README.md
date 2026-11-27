# Restored-Q circuit rerun — historical failure

This 26 November fourteen-corner test uses `Q = diag([.02 .02 .005 .10 .10].^2)`, ABS/TC on, sensors and estimation. It stops at 6.046 s when absolute path error exceeds 10 m. This is a **failure time**, not a lap time. Sensor noise, R, P0 and estimator equations were not changed.

[Video](Circuito_Q_restituida_ABS_TC_2D.mp4) · [Diagnostics](Diagnostico_Q_restituida.png) · [Metrics](resumen.csv) · [Histories](lazo_completo.mat)

The renderer replays these saved histories. The original run used the previous instantaneous ABS. The current package subsequently returned to the successful reduced-Q configuration by user decision; these results must not be relabelled as a run of the new modulated ABS.
