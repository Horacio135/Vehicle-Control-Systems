# V03 — Círculo de adherencia

## Problema y decisión

Evitar que Fx y Fy utilicen por separado toda la misma capacidad de contacto.

## Implementación

Un factor común limita la resultante. La telemetría permite observar uso de adherencia y saturación en giro.

Fuente: [CuatroruedasV3_simulacion_circulo_adherencia.mlx](../../../Dynamic%20Model/CuatroruedasV3_simulacion_circulo_adherencia.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Se conservan cinco figuras: skidpad y telemetría a 5 y 30 m/s, y el círculo de adherencia. Sus etiquetas describen esas ejecuciones, no el único valor del MLX actual.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

La saturación por sí sola no representa ni bloqueo mecánico ni la caída pospico de un neumático real.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
