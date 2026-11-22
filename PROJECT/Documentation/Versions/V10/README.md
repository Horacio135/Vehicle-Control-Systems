# V10 — Circuito de estrés y conductor brusco

## Problema y decisión

Exponer los límites de la planta con curvas más exigentes y aplicación rápida del acelerador y freno.

## Implementación

Circuito de 631.56 m y 14 curvas; chicane; 90° R4 m tras recta de 172 m; 70° R3.5 m. Objetivos máximos con aplicación exponencial, 95 % en 90 ms de aceleración y 60 ms de frenado.

Fuente: [Prueba_hibrida_V10_Estres_14_curvas.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V10_Estres_14_curvas.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Se preservan campañas 6/10/14/18/22 y 15/20/25/30/35 m/s. A 35 m/s: 51.384 s, error máximo 1.263 m, bloqueo aproximado 12.276 % y CG fuera 0 %. Los cinco casos de ambas campañas completan vuelta.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

Completar no prueba estabilidad universal. La utilización y el bloqueo son magnitudes del modelo simplificado; no existe ESC que module la demanda.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
