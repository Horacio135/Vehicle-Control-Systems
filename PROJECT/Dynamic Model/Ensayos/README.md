# Plant manoeuvres

[← Dynamic model](../README.md) · [Braking results](../../Results/V8_Braking/README.md)

These functions receive parameters and function handles from the main Live Script. They are not standalone programs to run before initializing the plant. The historical directory name `Ensayos` is retained because existing scripts use it.

| File | Manoeuvre and purpose |
|---|---|
| `Simulacion_skidpad.m` | Plot and analyse a previously simulated circular manoeuvre |
| `Simulacion_skidpad_parado.m` | Circular manoeuvre starting from rest |
| `Simulacion_eslalon.m` | Periodic steering changes with an initial speed |
| `Simulacion_eslalon_parado.m` | Slalom starting from rest |
| `Simulacion_aceleracion_frenada.m` | Maximum demand to 35 m/s, then braking; distance, force and utilization |
| `Simulacion_frenada_giro.m` | Two steering demands, 2° and 10°, starting 4 m after braking begins |

Both braking tests use the complete 20-component state and `[z_next,tele]=pasoDinamico(z,u,p,dt)`. They include neither ABS nor automatic brake-release pulses. Lock-mode lateral-force removal can prevent both steering manoeuvres. Historical V7 is not necessarily compatible with these helper versions.
