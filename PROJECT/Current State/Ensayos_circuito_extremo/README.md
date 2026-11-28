# Extreme 33-corner scenarios

[Results and comparisons](../../Results/Extreme_33_Corners/README.md) · [Design rationale](../../Documentation/Design_Decisions/Extreme_33_Corners_2026-11-28.md)

This dedicated harness integrates ESC V1 with both perfect state and the complete noisy sensing/EKF loop. It reuses the existing core `Control`, `Dinamica`, `Configuracion`, `Sensores` and `Estimacion` directories in Current State; none of those eighteen files changed. Do not add historical folders recursively to the MATLAB path.

## Recommended: complete estimated loop

From the repository root in MATLAB R2026a:

```matlab
run(fullfile('PROJECT','Current State','Ensayos_circuito_extremo', ...
    'Lazo_completo_sensores_estimador','Ejecutar_lazo_completo_33.m'));
run(fullfile('PROJECT','Current State','Ensayos_circuito_extremo', ...
    'Lazo_completo_sensores_estimador','Verificar_lazo_completo_33.m'));
addpath(fullfile('PROJECT','Current State','Ensayos_circuito_extremo','Lazo_completo_sensores_estimador'));
Renderizar_lazo_completo_33_2D;
```

The renderer can also use the published MAT when no new local simulation exists. Results of new runs go to `Current State/Resultados/Lazo_completo_33_curvas`, preserving published evidence. The default fourteen-corner Live Script is still a separate ABS comparison; it has not silently gained ESC.

## Other retained scenarios

- `Ejecutar_circuito_extremo.m`: four perfect-state cases, no aids / ABS only / TC only / ABS+TC+ESC. The first three terminate on loss of control; no restart or pose reset.
- `Comparacion_cenital_ESC/Ejecutar_circuito_extremo.m`: ABS+TC in both cases, with/without ESC. A temporary driver recovery manoeuvre permits continuation after loss of control. Once recovered, the original aggressive profile is restored without a permanent low-speed cap. The final run requires a return inside the lane before completion.
- `Comparacion_cenital_ESC/Renderizar_cenital.m`: fixed top-down whole-track view, x8. Reads newly generated histories or reconstructs the saved context from two case MATs.
- `Comparacion_cenital_ESC/Renderizar_telemetria_ESC.m`: perfect-state controlled car in 2D/3D, x1, with intervention, force and capacity plots. These videos do **not** include sensors/EKF. Pass `'2D'` or `'3D'`; the 3D scene is a visual extrusion of planar dynamics.

The track, profile and original Pure Pursuit law are common. `solicitarCorreccionESC` preserves the original ESC reference, activation, gains, wheel selection and ramps; it exposes the selective brake request before ABS. `coordinarAyudasExtremo` combines nominal and selective braking, then applies one final ABS modulation. During ESC intervention motor demand is removed because the plant rejects mixed driving/braking wheel signs. The external longitudinal anti-windup is unchanged between compared controlled cases.

Control in the complete loop receives only estimates. The capacity adapter uses controller-owned memories rather than true tyre memories; ground truth is reserved for sensor synthesis, the declared known initial condition, evaluation and rendering. No obstacles are present, so emergency control is inactive.

MATLAB source identifiers/comments and original recorded labels remain Spanish. Surrounding documentation is English.
