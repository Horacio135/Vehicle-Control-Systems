# ESC V1: theory, implementation and scope

[Runnable scenario](../Current%20State/Ensayos_ESC/README.md) · [Recorded evidence](../Results/ESC_V1_13m/README.md)

## Control objective and reference

ESC V1 requests a corrective yaw moment through wheel braking. Pure Pursuit remains the source of steering; this ESC does not add active steering. Propulsion is removed while braking or its release is active, matching the hybrid plant's prohibition on simultaneous positive and negative wheel-force commands.

The reference uses a steady-state linear bicycle relation, with `Cf` and `Cr` denoting total axle cornering stiffnesses:

```math
K_u=\frac{m}{L}\left(\frac{b}{C_f}-\frac{a}{C_r}\right),\qquad
r_0=\frac{v_x\delta}{L+K_uv_x^2}.
```

The reference is limited to `0.85 min(mu) g / max(vx,2)` and filtered with an 80 ms time constant. Here `delta` is the equivalent road-wheel angle, not an unconverted steering-wheel angle. The controlled error is `er = rref − r`; vehicle sideslip is `beta = atan2(vy,max(vx,0.1))`.

## Activation and moment demand

Entry occurs when `|er| > 0.15 rad/s` or `|beta| > 5°`. Release requires both `|er| < 0.06 rad/s` and `|beta| < 2°`. Activation is disabled below 2 m/s. During intervention:

```math
M_{z,req}=1800\,e_r+2500\,\beta,
\qquad |M_{z,req}|\leq1000\ \mathrm{Nm}.
```

The sideslip sign follows the package's X-forward/Y-left convention. These gains and thresholds are initial simulation choices, not identified Bosch calibration or guarantees for another vehicle.

## Wheel selection and actuation

Front-wheel candidates are used for same-direction oversteer or excessive sideslip; rear-wheel candidates otherwise. Selection checks the sign and magnitude of the direct braking moment arm:

```math
\ell_i=y_i\cos\delta_i-x_i\sin\delta_i,\qquad
\Delta M_{z,i}=b_i\ell_i,\quad b_i\geq0.
```

The largest valid lever arm determines the selected wheel; requested braking is limited to 1800 N. Application/release time constants are 15/20 ms, followed by the existing modulated force-based ABS. The selection is a simple heuristic, not an optimal allocation. Changed lateral tyre force also contributes to the actual yaw response, so the direct moment expression alone does not predict the whole vehicle response.

There is no path-curvature speed planner or common preventive deceleration request in this controller. Its reference speed remains 20 m/s; actual speed decreases as an outcome of stabilization. The driver sends no negative wheel-force demand. The original physical model and ABS/TC blocks remain intact.

## Relationship to real ESC

Bosch describes comparison of desired and actual motion followed by powertrain/brake intervention. Its technical teaching material separates reference generation, yaw-moment regulation and wheel-level allocation. Vehicle sideslip estimation is important when yaw rate alone does not describe stability. These principles motivate the structure, not its numerical calibration. [Bosch ESP overview](https://www.bosch-mobility.com/en/solutions/braking-systems/electronic-stability-program/), [Bosch technical lecture, pp.24–43](https://moodle.hs-kempten.de/pluginfile.php/41757/mod_resource/content/1/02_Bremsbasierte-Assistenzsysteme_ESP_HSK_WSS21-22.pdf), [van Zanten, public SAE abstract](https://saemobilus.sae.org/papers/bosch-esp-systems-5-years-experience-2000-01-1633).

## Current evidence and next integration work

In the 13 m-transition test, maximum sideslip falls from 87.8843° to 1.8809°. The baseline aborts at 3.810 s; the controlled car completes at 7.052 s, with three cone contacts. Stabilization is demonstrated in this case; cone-clearance success is not.

The moose benchmark supplies perfect state and physical capacity memories explicitly. The later [33-corner complete-loop harness](../Current%20State/Ensayos_circuito_extremo/README.md) now supplies estimated motion and controller-owned capacity prediction, combining nominal and selective braking before one final ABS call. It completes in 191.074 s versus 190.006 s with perfect state. The default fourteen-corner entry remains separate; multiple noise seeds and targeted validation are still needed. No active steering or hydraulic pressure plant has been added. The 3D recording is a visualization of the same planar histories, not an additional dynamic simulation.
