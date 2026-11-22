# Ensayos de la planta V8

Los scripts reciben parámetros y manejadores de funciones desde el Live Script principal; no son seis programas independientes para ejecutar sin inicializar la planta.

| Archivo | Maniobra y finalidad |
|---|---|
| `Simulacion_skidpad.m` | Gráficas y análisis del giro circular ya simulado |
| `Simulacion_skidpad_parado.m` | Giro circular arrancando desde reposo |
| `Simulacion_eslalon.m` | Cambios periódicos de dirección con velocidad inicial |
| `Simulacion_eslalon_parado.m` | Eslalon desde reposo |
| `Simulacion_aceleracion_frenada.m` | Demanda máxima hasta 35 m/s y posterior frenada; distancia, fuerza y utilización |
| `Simulacion_frenada_giro.m` | Dos órdenes de dirección (2° y 10°), iniciadas tras recorrer 4 m frenando |

Las dos pruebas de frenada usan veinte componentes de estado y `[z_next,tele]=pasoDinamico(z,u,p,dt)`. No incorporan ABS ni liberaciones automáticas del freno. La anulación lateral durante el bloqueo puede impedir ambos giros. Los scripts antiguos V7 no tienen por qué ser compatibles con estas versiones.
