# Hybrid tests

[← Project](../README.md) · [Results](../Results/README.md) · [Videos](../Results/Hybrid_Tests/README.md) · [History](../Documentation/Versions/README.md)

**V8 dynamics combined with Pure Pursuit and longitudinal control.** The tracking basis comes from coursework practice 4; these scenarios do not yet implement ESC. Each Live Script includes local functions to avoid depending on the original university directory.

## Choose a scenario

| Stage/file | Archived configuration | Purpose |
|---|---|---|
| V8 PurePursuit, animated/non-animated | Current files: 6 and 10 m/s | First eight-corner circuit |
| V9 PurePursuit | 6, 10, 14, 18, 22, 26 m/s | Speed planner; historical plotting limitation |
| V9 PurePursuit animated | 6, 10, 14, 18, 22 m/s | Five independently planned cases |
| V10 Estres 14 curvas | 15, 20, 25, 30, 35 m/s | Chicane, tight corners and abrupt driver demands |
| V11 Mapeo limite | 25, 30, 32, 35, 40 m/s | Baseline comparison and later braking |
| V11 35ms 3D | One 35 m/s case | 3D display of the same planar dynamics |
| V12 35ms 3D Aceleracion agresiva | One 35 m/s case | Doubled driving-force request and faster application |

Earlier videos use different configurations. Check [Results](../Results/README.md) before comparing lap times. Historical aliases such as `hist15` or `hist20` are not reliable indicators of the speed cap; use `resultados{i}.v_set`.

## Speed planning

`planificarVelocidad` calculates the curvature limit `sqrt(ayMax/abs(kappa))`, applies `vMax`, and propagates braking/acceleration assumptions through backward/forward sweeps of v², including circuit closure. The lateral planning limit is an assumption, not a change to plant mu.

`consultarPerfil` uses `mod(s,longitud)` and interpolates v². It is required **on the first lap too** to provide a continuous reference and handle closure; it is not only for subsequent laps. Speed-reference anticipation and Pure Pursuit lookahead are separate parameters. Tracking uses `Ld=p.Ld0+p.kLd*max(vx,0)`, not a fixed number of future samples.

| Parameter | V9 | V10 | V11/V12 |
|---|---:|---:|---:|
| `plan.ayMax` [m/s²] | 6 | 8 | 11 |
| `plan.axAcel` [m/s²] | 2.5 | 6 | 6 |
| `plan.axFreno` [m/s²] | 3.5 | 8 | 12 |
| `plan.anticipacionPI` [s] | 0.30 | 0.10 | 0 |

V11/V12 first define baseline parameters for `planBase`, then overwrite them. The rightmost column lists the effective demanding-plan values.

## Stress circuit and driver

V10–V12 start from rest. The 631.56 m circuit has 14 corners, a chicane at corners 4–5, a 172 m straight before the 90° corner (R=4 m), and a 70° corner (R=3.5 m).

Sufficient speed error triggers maximum requested force. The PI handles near-reference operation and its integral is frozen during full-demand interventions. Applied demand approaches its target exponentially rather than through an ideal step.

| Driver setting | V10/V11 | V12 |
|---|---:|---:|
| Maximum driving-force request [N] | 2500 | 5000 |
| Maximum braking request [N] | 5000 | 5000 |
| Acceleration time constant [s] | 0.030 | 0.010 |
| Braking time constant [s] | 0.020 | 0.020 |
| Approximate 95% acceleration application [ms] | 90 | 30 |

## Animation

The 2D players show body orientation and global velocity. V11/V12 3D use `patch`, `hgtransform`, `campos` and `camtarget`: a volumetric car, cones, steered front wheels and chase/overview cameras. Wheel rotation, roll, pitch and dynamic body height are not invented.

Closing the player deletes its timer. For batch execution, set `ensayo.animar=false` or `op3D.reproducir=false` **in a temporary copy**. Do not use `executeAndSave` with autoplay still active. Existing MP4 files are indexed in [Hybrid_Tests](../Results/Hybrid_Tests/README.md).

<details>
<summary>Historical V9 plotting issue</summary>

The non-animated V9 has six cases but some figures still use `tiledlayout(2,2)` and four-case aliases. Plotting can fail after simulation results have been calculated. It is preserved intact; use animated five-case V9 or the four-case [Run_Sources](../Results/Run_Sources/README.md) snapshot. Neither historical behaviour nor plant physics was silently corrected during publication.

</details>
