# Official 33-corner complete-loop result

[Campaign and perfect-state comparison](../README.md) · [Watch the gallery](https://horacio135.github.io/Vehicle-Control-Systems/#current)

The official [2D recording](Full_loop_33_corners_2D_actual_estimated.mp4) includes sensors, the seven-state EKF, estimated-position speed planning, Pure Pursuit, longitudinal control and force-based ABS/TC plus ESC V1. Blue is the actual car; the orange dotted outline uses estimated X, Y and yaw with the same footprint geometry. It is not an uncertainty envelope or a second vehicle.

The lap completes in 191.074 s; maximum actual centreline error is 0.504636 m and Euclidean position-estimation RMSE is 0.153999 m. Effective interventions total 17.540 s ABS, 118.030 s TC and 25.042 s ESC. The video is normal speed, 20 fps; frame quantization makes its duration slightly longer than the physical history.

`Lazo_completo_33_curvas.mat` retains `h`, `p`, `plan`, `pista`, `perfil`, `ayudas` and `resumen`, including actual/estimated states, raw measurements, covariances, orders and wheel forces. `Resumen.json` and `Verificacion_lazo.json` contain numerical summaries and configuration checks. The eighteen core block files are unchanged from the existing package. No obstacles are introduced; emergency intervention is inactive.

The unchanged Q and noise settings are listed in the [campaign page](../README.md). Control never reads physical tyre memories; the existing external capacity model/adapter provides deterministic controller-owned memories from estimates and known previous orders. The state used to initialize the estimator is the declared known initial condition. Subsequent ground truth is used only for sensor synthesis, plant evolution, evaluation and rendering.

This video is the same byte-for-byte recording previously offered as temporary mobile access. It is now official evidence, not an updated physics run. Published labels remain Spanish; repository documentation is English.
