# V02 — Telemetría de cuatro ruedas

## Problema y decisión

Poder observar las fuerzas de cada rueda y relacionarlas con la trayectoria.

## Implementación

pasoDinamico devuelve telemetría además del estado; se amplían registros y gráficas del ensayo circular.

Fuente: [CuatroruedasV2_simulacion_4ruedas.mlx](../../../Dynamic%20Model/CuatroruedasV2_simulacion_4ruedas.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Se preservan el MLX y su código exportado. El archivo configura un skidpad de 18 m/s; ese valor no debe atribuirse a todas las figuras históricas.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

Aún se necesita compartir el límite entre fuerza longitudinal y lateral. No se han calculado velocidades angulares de rueda.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
