# Dynamic Model

La referencia actual es **CuatroruedasV8_bloqueo_ruedas.mlx**. Los prototipos y V1–V7 se conservan para reconstruir las decisiones, sin alterar sus ecuaciones ni adaptar sus interfaces durante esta publicación.

## Evolución de la planta

| Etapa | Incorporación | Evidencia disponible |
|---|---|---|
| V1 | Movimiento plano, fuerzas en cuatro contactos, comparación cinemática/dinámica; límites Fx/Fy separados | Código y salidas incrustadas del MLX |
| V2 | Telemetría individual de las ruedas | Código y salidas del MLX |
| V3 | Círculo compartido de adherencia longitudinal/lateral | [Figuras](../Results/V3_Friction_Circle/README.md) |
| V4 | Transferencias longitudinal/lateral y rigidez dependiente de carga | [Figuras](../Results/V4_Load_Transfer/README.md) |
| V5 | Pérdida/recuperación lateral trasera por exceso de tracción | [Figuras y vídeos](../Results/V5_Slip_Tests/README.md) |
| V6 | Proyección de velocidades al marco de cada rueda | Código y salidas del MLX |
| V7 | Separación de ensayos y organización de funciones | Código histórico, con dependencia de interfaces anteriores |
| V8 | Frenado con modo de bloqueo aproximado, recuperación y estado completo | [Pruebas de frenado](../Results/V8_Braking/README.md) |

La explicación de cada decisión está en [Documentation/Versions](../Documentation/Versions/README.md).

## Interfaz congelada

```matlab
[z_next, tele] = pasoDinamico(z, u, p, dt);
u = [Fx_total_solicitada; delta_rueda];
```

`Fx` es **fuerza [N]**, no potencia; es positiva al acelerar y negativa al frenar. `delta` está en radianes y corresponde a las ruedas delanteras, no al volante convertido por una relación de dirección. `p` contiene parámetros de masa, geometría, neumáticos, límites y constantes de tiempo. `dt` se expresa en segundos.

Orden de ruedas: **DI, DD, TI, TD**. Reparto actual de tracción `[0 0 .5 .5]`; frenada `[.3 .3 .2 .2]`. No existe todavía una orden de frenado independiente por rueda.

| Índices de z | Significado |
|---|---|
| 1:6 | X [m], Y [m], psi [rad], vx [m/s], vy [m/s], r [rad/s] |
| 7:8 | Aceleraciones físicas del paso anterior para transferencia de carga |
| 9:12 | Memoria `eta_slip` por rueda |
| 13:16 | Memoria `eta_brake` por rueda |
| 17:20 | Modo de bloqueo aproximado por rueda, 0/1 |

Son seis variables de movimiento y catorce memorias, no veinte grados de libertad mecánicos. La inicialización de seis componentes añade aceleraciones cero, factores de adherencia uno y bloqueo cero. Las llamadas directas a `pasoDinamico` requieren el vector completo de veinte componentes.

`tele` devuelve, entre otros campos, `Fxw`, `Fyw`, `Fzw`, `Fmax_w`, `Fmax_peak_w`, `lambda_w`, `alpha_w`, `Caw`, `eta_slip`, `eta_brake`, `bloqueo_w`, fuerzas en el chasis y aceleraciones. Las fuerzas `Fxw/Fyw` están en ejes de rueda; para balances globales hay que usar su transformación al chasis. Telemetría de un paso y estado siguiente no deben confundirse temporalmente.

## Fuerzas y adherencia

La carga vertical se redistribuye cuasiestáticamente con las aceleraciones anteriores. La rigidez lateral escala con carga mediante `Caw=Caw0.*(Fzw./Fzw0).^nCa`. El círculo aplica un factor común `lambda=min(1,Fmax/hypot(Fx_sol,Fy))`, y la pérdida lateral trasera usa una memoria adicional dependiente del exceso de demanda motriz.

En frenado, la entrada/salida del modo de bloqueo se decide mediante la relación entre demanda y capacidad **nominal**, con histéresis. El objetivo de capacidad reducida es:

```matlab
eta_obj = brake_slide_ratio + (1-brake_slide_ratio)*exp(-abs(vx_local)/v_brake_slide);
mu_eff = mu_base * eta_brake;
```

`eta_brake` converge exponencialmente al objetivo; la constante es distinta al perder adherencia, aliviar parcialmente o soltar completamente. Valores de ensayo: ratio mínimo 0.68, escala de velocidad 15 m/s, pérdida 0.03 s, recuperación parcial 0.10 s y liberación completa 0.01 s. Son parámetros ilustrativos pendientes de calibración.

En el modo especial de bloqueo se anula `Fy` y no se rota la fuerza longitudinal con la dirección: el frenado actúa según X del chasis. Esto evita conservar artificialmente dirección por `Fx*sin(delta)`. El factor longitudinal unitario conserva el límite de frenado reducido; **no significa que Fx sea 1 N ni que desaparezca la saturación**. Es una decisión deliberadamente simplificada del ensayo.

## Ejecución y compatibilidad

Seleccionar esta carpeta como Current Folder y ejecutar el V8 `.mlx`. Los seis scripts de [Ensayos](Ensayos/README.md) corresponden a la interfaz actual de V8. No ejecutar V7 con estos scripts como si sus firmas fueran intercambiables: su `pasoDinamico` aún exige memorias separadas y el ensayo de recta actual espera cuatro entradas.

Los modelos V1–V6 son archivos históricos autocontenidos. Sus parámetros y firmas difieren. No se han vuelto a ejecutar todos en esta publicación ni se promete que todas las figuras antiguas se regeneren con las versiones actuales de los scripts.

## Límites físicos que se mantienen

No hay velocidades angulares, pares ni inercia de rueda; por tanto, `bloqueo_w` no demuestra bloqueo mecánico y `eta_slip` no es un slip ratio real. No hay suspensión dinámica, balanceo, cabeceo ni modelo experimental de neumático. La suma vertical mg y el círculo son comprobaciones internas, no calibración experimental. Una frenada no tiene un límite universal de 1 g: depende de adherencia, carga y resistencias.

La convención de deriva merece una revisión futura: V6–V8 proyectan velocidades al marco girado de rueda y calculan `delta-atan2(vy_local,vx_local)`. Esa expresión puede volver a introducir el ángulo de dirección; se conserva por la decisión de congelar la planta y debe contrastarse antes de atribuir validez física a los resultados.
