# V06 — Cálculo individual en ejes de rueda

## Problema y decisión

Expresar velocidades y fuerzas del contacto en el marco orientado de cada rueda.

## Implementación

Se proyectan vxw y vyw al marco local, conservando fuerzas individuales, saturación, cargas y pérdida lateral.

Fuente: [CuatroruedasV6_dinamica_indiv_rueda.mlx](../../../Dynamic%20Model/CuatroruedasV6_dinamica_indiv_rueda.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Código y salidas del MLX conservados. No se publicó un juego independiente de métricas V6; no se extrapolan las de V5 o V8.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

Revisar en una fase futura la convención delta-atan2(vy_local,vx_local), por posible doble introducción de delta. La publicación no altera la física.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
