# V01 — Modelo dinámico inicial

## Problema y decisión

Comparar las respuestas cinemática y dinámica manteniendo un chasis plano.

## Implementación

Se calculan fuerzas por contacto, transformaciones al chasis y momento de guiñada, con resistencias aerodinámica y de rodadura. Fx y Fy aún se limitan separadamente.

Fuente: [CuatroruedasV1_simulacion.mlx](../../../Dynamic%20Model/CuatroruedasV1_simulacion.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

El código y las salidas incrustadas del MLX son la evidencia conservada; no hay una campaña numérica tabulada independiente de V1. No se asignan valores de error inventados.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

Los límites independientes no representan correctamente el uso simultáneo de adherencia; esta observación conduce a V3.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
