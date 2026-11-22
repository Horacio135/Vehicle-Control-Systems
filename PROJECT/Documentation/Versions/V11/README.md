# V11 — Mapeo al límite y visualización 3D

## Problema y decisión

Reducir menos la velocidad en curva y retrasar el frenado sin cambiar la planta.

## Implementación

ayMax=11, axFreno=12 m/s², anticipacionPI=0; se conserva axAcel=6. Se guarda comparación con V10 y una variante de un coche de 35 m/s en 3D.

Fuente: [Prueba_hibrida_V11_Mapeo_limite.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V11_Mapeo_limite.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

A 35 m/s: 47.680 s, error máximo 0.776 m, bloqueo 19.869 %, CG fuera 0 %. En la recta inicial se planifica frenar 25.5 m más tarde que la base. La campaña actual incluye 25/30/32/35/40 m/s; el caso 40 solo alcanza un pico vx de 37.405 m/s.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

La mejora observada es específica de estas condiciones y aumenta el bloqueo aproximado. La visualización 3D conserva la dinámica plana y no añade rotación de ruedas.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
