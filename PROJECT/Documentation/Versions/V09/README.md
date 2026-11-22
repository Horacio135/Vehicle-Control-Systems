# V09 — Planificador de velocidad por curvatura

## Problema y decisión

Acelerar en rectas y reducir velocidad antes de curvas, frente a la referencia constante del primer híbrido.

## Implementación

Se combinan límites por curvatura, barridos de v², consulta periódica de perfil y PI longitudinal. Se ensayan 14/18/22/26 y después 6/10/14/18/22 m/s.

Fuente: [Prueba_hibrida_V9_PurePursuit_animada.mlx](../../../Hybrid%20Tests/Prueba_hibrida_V9_PurePursuit_animada.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Los cinco casos animados completan vueltas. Tiempos de 44.164, 31.628, 30.436, 30.432 y 30.432 s. Aumentar el techo por encima de 14 m/s aporta poco en ese circuito porque limita el perfil por curvatura.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

El V9 no animado actual tiene seis casos y figuras 2x2; se conserva con advertencia. El archivo animado de cinco casos es el punto de entrada recomendado.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
