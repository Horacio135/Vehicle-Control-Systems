# Hybrid Tests — seguimiento, planificación y estrés

Estos Live Scripts combinan la planta V8 con Pure Pursuit y control longitudinal. La base de seguimiento procede de la práctica 4; no son aún un ESC. Cada archivo incorpora sus funciones locales para no depender del directorio universitario original.

| Archivo/etapa | Configuración conservada | Finalidad |
|---|---|---|
| V8 PurePursuit, con/sin animación | Archivo actual: 6 y 10 m/s | Primer circuito de ocho curvas |
| V9 PurePursuit | 6, 10, 14, 18, 22, 26 m/s | Planificador de velocidad; archivo histórico con limitación en figuras |
| V9 PurePursuit animada | 6, 10, 14, 18, 22 m/s | Cinco casos con perfiles independientes |
| V10 Estres 14 curvas | 15, 20, 25, 30, 35 m/s | Chicane, curvas cerradas y conductor brusco |
| V11 Mapeo limite | 25, 30, 32, 35, 40 m/s | Comparación de mapeo base con frenado más tardío |
| V11 35ms 3D | Un caso de 35 m/s | Visualización 3D de la misma dinámica plana |
| V12 35ms 3D Aceleracion agresiva | Un caso de 35 m/s | Demanda motriz duplicada y aplicación más rápida |

Los vídeos históricos conservan otras configuraciones. Consultar [Results](../Results/README.md) antes de comparar tiempos. Los alias `hist15`, `hist20`, etc. en algunos archivos históricos no son una fuente fiable del techo de velocidad: usar `resultados{i}.v_set`.

## Planificación de velocidad

`planificarVelocidad` calcula un límite por curvatura `sqrt(ayMax/abs(kappa))`, limitado por `vMax`, y hace barridos hacia atrás y hacia delante sobre v² para respetar las hipótesis de frenado y aceleración. El cierre del circuito también se incluye. El límite lateral es una hipótesis del planificador, no una modificación de mu en la planta.

`consultarPerfil` usa `mod(s,longitud)` e interpola v². Se necesita **también en la primera vuelta** para obtener una referencia continua y tratar el cierre; no sirve solo para vueltas adicionales. La anticipación temporal de velocidad y la distancia de mirada de Pure Pursuit son parámetros distintos. El seguimiento usa `Ld=p.Ld0+p.kLd*max(vx,0)`; no un número fijo de muestras por delante.

| Parámetro | V9 | V10 | V11/V12 |
|---|---:|---:|---:|
| `plan.ayMax` [m/s²] | 6 | 8 | 11 |
| `plan.axAcel` [m/s²] | 2.5 | 6 | 6 |
| `plan.axFreno` [m/s²] | 3.5 | 8 | 12 |
| `plan.anticipacionPI` [s] | 0.30 | 0.10 | 0 |

En V11/V12 aparecen primero los parámetros base para construir `planBase` y después sus sobreescrituras. Los valores efectivos son los de la última columna.

## Conductor y circuito de estrés

V10–V12 arrancan desde reposo. El circuito mide 631.56 m, con 14 curvas, chicane en las curvas 4–5, una recta de 172 m antes de la curva de 90° (R=4 m) y otra curva de 70° (R=3.5 m).

Con error de velocidad suficiente se solicita fuerza máxima; el PI se reserva para mantenimiento y se congela su integral en intervenciones a fondo. La demanda aplicada converge exponencialmente al objetivo, no mediante escalones. En V10/V11: 2500 N de tracción, 5000 N de frenada, tauAcel=0.030 s y tauFreno=0.020 s. En V12: tracción 5000 N y tauAcel=0.010 s; el resto se conserva. El 95 % de aplicación tarda aproximadamente tres constantes de tiempo.

## Animación

Los reproductores 2D muestran orientación y velocidad global. Los de V11/V12 3D usan `patch`, `hgtransform`, `campos` y `camtarget`: carrocería con volumen, conos, ruedas delanteras orientadas y cámara de seguimiento/general. No se anima giro de rueda como si se hubiera calculado, ni se añaden balanceo, cabeceo o elevación de la carrocería.

Cerrar la ventana elimina el temporizador. Para ejecutar en batch, cambiar **en una copia temporal** `ensayo.animar=false` u `op3D.reproducir=false`; no usar `executeAndSave` con una reproducción automática activa. Los MP4 existentes se encuentran en [Results/Hybrid_Tests](../Results/Hybrid_Tests/README.md).

## Archivo histórico con limitación conocida

El V9 no animado contiene seis casos pero algunas gráficas mantienen `tiledlayout(2,2)` y alias de cuatro casos. Puede fallar al graficar aunque la simulación haya calculado los resultados. Se conserva íntegro; usar el V9 animado de cinco casos o la instantánea de cuatro casos de [Run_Sources](../Results/Run_Sources/README.md). No se ha corregido silenciosamente el histórico ni la planta durante esta actualización.
