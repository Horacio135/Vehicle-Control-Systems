# V05 — Pérdida lateral por exceso de tracción

## Problema y decisión

Mostrar cómo una solicitud motriz excesiva deteriora la capacidad de guiar el eje trasero.

## Implementación

Se introduce eta_slip como memoria por rueda y una pérdida adicional en las traseras cuando se supera la capacidad. La recuperación es progresiva.

Fuente: [CuatroruedasV5_simulacion_deslizamiento_longitudinal.mlx](../../../Dynamic%20Model/CuatroruedasV5_simulacion_deslizamiento_longitudinal.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

La carpeta V5_Slip_Tests conserva una figura y tres vídeos de derrape/skidpad/eslalon. Son evidencia visual del modelo aproximado.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

El nombre del archivo no implica que haya un slip ratio físico: no hay omega de rueda ni inercia rotacional.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
