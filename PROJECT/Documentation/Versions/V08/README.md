# V08 — Bloqueo aproximado, recuperación e interfaz

## Problema y decisión

Distinguir saturación normal de pérdida de capacidad al frenar en exceso y preparar la conexión del controlador.

## Implementación

Se añade un modo con histéresis, capacidad dependiente de velocidad, liberación rápida y fuerza lateral nula durante bloqueo. Las memorias se integran en un z de 20 componentes y la firma queda en cuatro entradas/dos salidas.

Fuente: [CuatroruedasV8_bloqueo_ruedas.mlx](../../../Dynamic%20Model/CuatroruedasV8_bloqueo_ruedas.mlx). La copia `.m` legible está en Source Exports, junto a los otros modelos de la misma familia.

## Simulaciones y resultados

Se conservan frenada hasta detenerse, liberación de 50 ms, dos giros tras 4 m frenando, y la comparación antes/después de reorganizar estados. La recta registrada alcanza 35.002 m/s y termina en 7.472 s incluyendo aceleración y frenado; esto no es el tiempo de frenado aislado.

Los artefactos están indexados en [Results](../../../Results/README.md). Las métricas numéricas recientes proceden de [summary_all_runs.csv](../../../Results/Numerical/summary_all_runs.csv), calculadas sobre los registros completos; los CSV temporales se conservan aproximadamente a 20 Hz.

## Interpretación y límites

El modo no calcula bloqueo real. Se mantiene sin ABS/TCS y sin actuación independiente por rueda. La integración híbrida V8 introduce Pure Pursuit sin cambiar la planta.

La evidencia de cada etapa se conserva con su configuración. No se ha ejecutado de nuevo toda la historia ni se atribuyen las cifras de una versión a otra. Véanse [las decisiones transversales](../../Design_Decisions/README.md).
