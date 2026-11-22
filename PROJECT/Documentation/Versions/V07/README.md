# V07 — Separación de ensayos

## Problema y decisión

Separar maniobras y presentación del núcleo para preparar una arquitectura modular.

## Implementación

Se incorporan scripts externos en Ensayos y se mantienen las memorias anteriores como entradas adicionales del paso dinámico.

Fuente: [CuatroruedasV7_estructurado.mlx](../../../Dynamic%20Model/CuatroruedasV7_estructurado.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

El archivo documenta el estado anterior a la normalización. Los ensayos actuales corresponden a V8 y no son un entorno histórico V7 completo.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

Su pasoDinamico exige ax_prev, ay_prev y eta_prev; la recta actual espera solo z,u,p,dt. Se archiva sin prometer ejecución integral con dependencias actuales.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
