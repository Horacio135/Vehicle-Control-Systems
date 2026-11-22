# Numerical evidence

Se han exportado **50 historias** de los archivos MAT ya guardados, sin volver a simular ni modificar la planta. Una historia es una ejecución registrada: incluye casos base repetidos entre campañas, ensayos cinemáticos y dinámicos y pruebas de frenado; no son 50 experimentos independientes.

- [summary_all_runs.csv](summary_all_runs.csv): métricas calculadas sobre **todas** las muestras originales.
- [video_inventory.csv](video_inventory.csv): duración, fps y dimensiones comprobados con VideoReader.
- `V*_case*.csv` / `V*_hist*.csv`: estado y telemetría temporal con selección de muestras aproximadamente a **20 Hz**, siempre incluyendo la última.
- `V*_config.json`: parámetros de vehículo/plan/ensayo cuando estaban presentes en el MAT original. La ausencia de un parámetro no se rellena con el de otra campaña.
- [raw_evidence_manifest.json](raw_evidence_manifest.json): tamaño y SHA256 de los MAT de procedencia, que no se duplican aquí.

La integración original usa aproximadamente 0.002 s; los CSV temporales son una reducción de datos para inspección. No usar su muestreo para medir picos de 10–30 ms: los extremos y porcentajes del resumen se calcularon antes de reducir. Los archivos MAT originales, grandes y parcialmente duplicados, se conservan localmente; los MLX archivados, instantáneas y parámetros permiten regenerar campañas.

## Columnas y unidades

`t_s`, `X_m`, `Y_m`, `psi_rad`, `vx_ms`, `vy_ms`, `r_rads` y memorias de z. `Fx_sol` y `Fxw/Fyw/Fzw/Fmax_*` están en N; `delta` en rad; `ax/ay` en m/s²; `eta_*`, `lambda_w` y utilización son adimensionales. Los sufijos de cuatro ruedas siguen DI, DD, TI, TD. `vref` es la referencia del controlador, no la velocidad real.

`peak_vx_ms` es el pico longitudinal; `peak_speed_ms` usa hypot(vx,vy). `max_lateral_error_m` y RMS usan ect; `CG_outside_pct` usa el criterio registrado para el centro de gravedad. `lock_pct` cuenta muestras con alguna rueda en modo de bloqueo. `saturated_pct` cuenta alguna lambda<1-1e-6. No es un porcentaje de deslizamiento físico.

`complete=1` significa vuelta completada si existe `completa`, o parada alcanzada en las pruebas que registran `paradaAlcanzada`. `complete=0` es un criterio registrado no alcanzado. **NaN significa no registrado**, no fallo. Las maniobras antiguas no tienen siempre un criterio de vuelta comparable al circuito híbrido. `stop_reason` conserva el motivo disponible.

## Campañas

| Dataset | Configuración |
|---|---|
| V08_standard | Skidpad, eslalon, arranques y recta; comparación de interfaz |
| V08_braking_turn | Giro de 2°/10° tras 4 m de frenada |
| V08_hybrid | Primera prueba: 6 y 14 m/s; el caso 14 pierde seguimiento |
| V09_four_caps | 14, 18, 22, 26 m/s |
| V09_five_caps | 6, 10, 14, 18, 22 m/s |
| V10_five_caps | 6, 10, 14, 18, 22 m/s |
| V10_current_caps | 15, 20, 25, 30, 35 m/s |
| V11_initial_comparison | 15, 20, 25, 30, 35 m/s; base y límite |
| V11_current_caps | 25, 30, 32, 35, 40 m/s; historias guardadas del mapeo exigente |
| V11_single_3D | Un coche, techo de 35 m/s |
| V12_aggressive_3D | Un coche de 35 m/s, demanda extrema y su base |

La primera prueba híbrida guardada (6/14) no coincide con el archivo V8 actual (6/10). Se mantiene esa diferencia y no se atribuye el fallo de 14 m/s al caso de 10 m/s. Las gráficas y vídeos tienen sus propios conjuntos en sus README.
