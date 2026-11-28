# Controlled-car telemetry videos — perfect state

[Campaign overview](../README.md)

The [2D](ESC_ABS_TC_2D_telemetria_x1.mp4) and [3D](ESC_ABS_TC_3D_telemetria_x1.mp4) recordings use the same **190.006 s perfect-state controlled history**, not the later 191.074 s sensing/EKF run. Both play at normal speed, 20 fps. The 3D camera follows X/Y with fixed height and orientation; planar physics is not extended to roll, pitch or suspension.

Panels show effective aid intervention (not mere enablement), longitudinal driver demand, summed corrected requests, summed transmitted tyre Fx, effective combined capacity for each wheel and Pure Pursuit's requested angle. Pure Pursuit outputs steering, not a force. Wheel-coordinate Fx sums are not automatically chassis `m*ax`; steering rotation and resistances also matter. Fmax is combined capacity, not remaining longitudinal reserve after lateral use.

Requested force becomes an equivalent motor torque in traction. A 60 kW plant-side power limit and wheel inertia/Pacejka can separate corrected demand from actual contact force. During braking, the friction circle may reduce longitudinal transmission when lateral force is also demanded. These are not an invented plotting time delay; nominal/corrected/actual samples use the saved physical clock. These parameters remain illustrative, not experimentally identified.

Activation traces retain the 2 ms samples. Eight-second panels are precomputed for rendering, then masked to reveal only past data with a synchronized cursor. No future curve is deliberately displayed. Per-wheel nominal/corrected/actual force histories are in `Fuerzas_por_rueda.png`; full-lap intervention history is in `Activacion_y_fuerza_vuelta_completa.png`. `Verificacion_videos.json` records source SHA-256 and MP4 decoding checks. Its source corresponds to `../ESC_Recovery/Caso_2.mat`.
