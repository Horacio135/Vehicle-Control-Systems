# V12 — Solicitud de aceleración extrema

## Problema y decisión

Exigir más fuerza y más rápidamente incluso si se deteriora la salida de curva.

## Implementación

Fuerza motriz solicitada máxima 2500→5000 N y tauAcel 0.030→0.010 s (95 % en 30 ms). Se conservan frenada, mapeo, circuito, seguimiento y ecuaciones de la planta.

Fuente: [Prueba_hibrida_V12_35ms_3D_Aceleracion_agresiva.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V12_35ms_3D_Aceleracion_agresiva.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Vuelta 149.036 s, error máximo 4.347 m, RMS 1.258 m y CG fuera del carril 18.103 %. El factor lateral trasero mínimo baja de 0.153 a 0.078. El exceso longitudinal trasero durante aceleración sube de 49.664 a 98.556 %. El MP4 completo dura 149.133 s a 15 fps.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

No es una mejora de prestaciones. Se trata de un caso extremo de pérdida de adherencia útil para evaluar el futuro control; la salida de carril no se oculta ni corrige con cambios físicos.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
