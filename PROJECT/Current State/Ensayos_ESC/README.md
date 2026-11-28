# Selective-braking ESC V1 scenario

[Current package](../README.md) · [Results and approved video](../../Results/ESC_V1_13m/README.md) · [Theory](../../Documentation/ESC_V1.md)

```matlab
cd(fullfile('PROJECT','Current State','Ensayos_ESC'))
open('Ejecutar_alce_ESC.mlx')
% Run the entire Live Script: comparison, then a 2D rendering.
```

The scenario uses 13 m lane-change transitions, 4 m displacement, a 10 m offset straight, a 3 m corridor and a 20 m/s initial speed and reference. Pure Pursuit keeps the package's original parameters. The speed driver uses its maintenance PI, with negative requests suppressed. ESC V1 alone can request braking and removes propulsion while its wheel-braking intervention releases. External anti-windup freezes the speed integrator during that override; the longitudinal-controller source is unchanged.

Sensors and the estimator are bypassed explicitly. State and tyre-capacity memories are known in this benchmark. It is not a covert perfect-state fallback inside the estimated loop. The ESC operates on yaw-rate error and sideslip, with selective wheel braking followed by the existing ABS.

The two cases share all scenario parameters and differ only in ESC use. The uncontrolled car aborts after loss of control; the controlled car completes but still contacts three cones. See the full geometric verification in the results.

Future runs write to `Current State/Resultados/Alce_02_ESC_V1_transiciones_13m_2026-11-28`. Published evidence is stored separately in `PROJECT/Results/ESC_V1_13m`. To reproduce the selected 3D view, open that results directory's `Visualizacion_3D_ESC_V1_13m.mlx`; it replays the stored data without rerunning the plant.

This adds a dedicated test of `Control/controlESC.m` and `Configuracion/configuracionESC.m`. The default fourteen-corner `controlTotal` loop does not yet call this ESC. Dynamics, longitudinal/lateral controller equations, ABS, TC, sensors, estimator and Q are unchanged.
