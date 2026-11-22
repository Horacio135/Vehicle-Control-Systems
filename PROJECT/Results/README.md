# Results — simulaciones, figuras y vídeos

Se publican las evidencias existentes, incluidas las salidas desfavorables. No se han vuelto a renderizar los vídeos ni alterado los modelos para conseguir mejores trayectorias.

| Etapa | Evidencia |
|---|---|
| V1–V2 | Código y salidas incrustadas en los MLX de Dynamic Model; no hay métricas independientes tabuladas |
| [V3](V3_Friction_Circle/README.md) | Cinco PNG sobre círculo de adherencia, skidpad y telemetría |
| [V4](V4_Load_Transfer/README.md) | Nueve PNG de transferencia de cargas, rigidez y saturación |
| [V5](V5_Slip_Tests/README.md) | Una figura y tres vídeos de derrape/eslalon |
| V6–V7 | Código y salidas de MLX; limitaciones históricas documentadas |
| [V8](V8_Braking/README.md) | Frenado, liberación y giro con bloqueo aproximado |
| [V9](V9_Speed_Planner/README.md) | Perfiles, fuerzas y respuesta del seguimiento |
| [V10](V10_Stress/README.md) | Circuito de estrés y conductor brusco |
| [V11](V11_Speed_Mapping/README.md) | Mapeo más exigente y comparación con la base |
| [V11 3D](V11_3D/README.md) | Un coche con cámara móvil |
| [V12](V12_Aggressive_Acceleration/README.md) | Demanda de aceleración extrema y degradación del seguimiento |
| [Vídeos híbridos](Hybrid_Tests/README.md) | Siete MP4 de V8–V12 |
| [Numerical](Numerical/README.md) | Métricas de 50 historias y telemetría CSV |
| [Run_Sources](Run_Sources/README.md) | Instantáneas de código usadas en campañas históricas |

## Comparación de 35 m/s en el circuito de 14 curvas

| Configuración | Tiempo [s] | Error máximo [m] | RMS [m] | CG fuera [%] | Bloqueo [%] | Saturación [%] |
|---|---:|---:|---:|---:|---:|---:|
| V10 base | 51.384 | 1.263 | 0.497 | 0 | 12.276 | 45.429 |
| V11 exigente | 47.680 | 0.776 | 0.238 | 0 | 19.869 | 78.629 |
| V12 aceleración extrema | 149.036 | 4.347 | 1.258 | 18.103 | 3.225 | 99.361 |

Los porcentajes se calculan sobre muestras de tiempo: bloqueo si alguna rueda está en ese modo; saturación si alguna tiene lambda<1-1e-6; fuera de carril si el CG cruza el límite. No representan la envolvente de la carrocería. El porcentaje menor de bloqueo de V12 no demuestra mejor frenada: cambia mucho el tiempo total y predomina la pérdida de tracción.

## Lectura correcta de las conclusiones

1. El mapeo puede mejorar el tiempo en este circuito sin eliminar saturación o bloqueo aproximado.
2. Duplicar la demanda motriz no duplica la fuerza transferida. V12 muestra pérdidas graves de guiado y mucho peor tiempo de vuelta.
3. Un techo de 40 m/s no implica alcanzar 40 m/s: en V11 el pico registrado es 37.405 m/s.
4. Una animación 3D no valida dinámica vertical o rotacional no simulada.
5. Son resultados numéricos de un modelo fenomenológico. No se declara validación experimental ni funcionamiento de un ESC/ABS/TCS todavía inexistente.

Las versiones actuales pueden tener parámetros distintos de los que generaron un vídeo antiguo. Los nombres, configuraciones JSON e instantáneas permiten distinguirlos. No hay registros numéricos independientes de todas las versiones tempranas; esa ausencia se explicita en sus README.
