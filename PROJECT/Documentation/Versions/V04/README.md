# V04 — Transferencia de pesos

## Problema y decisión

Repartir la capacidad del vehículo de forma distinta durante aceleración, frenada y giro.

## Implementación

Se añaden transferencia longitudinal/lateral a partir de aceleraciones anteriores y rigidez lateral dependiente de Fz. Se amplían eslalon y arranque.

Fuente: [CuatroruedasV4_simulacion_transferencia_pesos.mlx](../../../Dynamic%20Model/CuatroruedasV4_simulacion_transferencia_pesos.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Nueve figuras conservan cargas, rigidez, aprovechamiento, skidpad y eslalon. Hay ensayos rotulados a 5 y 14 m/s. El equilibrio vertical y la respuesta de cargas son controles internos del modelo.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

Transferencia cuasiestática: no se introduce suspensión, balanceo ni cabeceo dinámico.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
