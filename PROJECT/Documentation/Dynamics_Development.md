# Development of the vehicle dynamics and complete simulation loop

[← Documentation](README.md) · [Current runnable package](../Current%20State/README.md) · [Post-V12 milestones](Versions/After_V12/README.md) · [Decision ledger](Design_Decisions/Post_V12_Decisions.md) · [Recorded results](../Results/README.md)

This report explains the progression from the first planar four-contact model to the current hybrid tyre model, Ackermann geometry and sensing–estimation–control loop. It distinguishes changes to physics from changes to the driver, track, controller and display. The snapshot date is **26 November 2026**.

The active plant uses longitudinal Pacejka only in the rear driving contacts, the selected historical braking approximation, load-dependent lateral stiffness and shared contact capacity. It is a useful numerical development platform, not an experimentally identified tyre or a validated real vehicle. The latest full-loop connection preserves the thirteen existing MATLAB block functions and uses external adapters. Documentation is English; MATLAB identifiers and comments remain Spanish as requested.

## Development sequence

| Stage | Problem addressed | Main decision |
|---|---|---|
| V1 | Compare kinematic and dynamic vehicle response | Planar chassis and forces at four contact locations |
| V2 | A trajectory alone hides contact behaviour | Return and log per-wheel telemetry |
| V3 | Independent Fx/Fy limits spend capacity twice | Shared friction circle |
| V4 | Contact capacity changes during manoeuvres | Quasi-static longitudinal/lateral load transfer |
| V5 | Excess motor demand did not adequately degrade guidance | Empirical rear lateral-loss memory |
| V6 | Forces and velocities require consistent wheel axes | Individual wheel-frame projections |
| V7 | Embedded manoeuvres were difficult to reuse | Separate test scripts and the plant |
| V8 | Saturation did not distinguish approximate wheel lock | Speed-dependent braking loss, release recovery and complete state |
| V9–V12 | Isolated manoeuvres did not expose circuit behaviour | Tracking, speed planning, stress geometry and aggressive demands |
| Post-V12 | Straight-line acceleration lacked physical wheel slip | Rotational rear wheels and longitudinal Magic Formula |
| Combined V2 experiment | Extend force–slip laws to all directions | Trial lateral/combined model; discard uncalibrated configuration after failures |
| Selected hybrid | Retain useful traction behaviour without accepting the failed combined configuration | Rear traction MF and restored historical braking |
| Ackermann | Parallel front steering was geometrically inconsistent | Individual front angles and one consistent slip-angle transform |
| Complete loop | Controllers still depended on plant information | Sensors, seven-state EKF and external controller-owned capacity prediction |

“Pacejka V1/V2” are names of later experiments; they do not replace the original V1–V12 numbering. The [version index](Versions/README.md) and [later milestones](Versions/After_V12/README.md) preserve that distinction.

## Planar chassis and coordinates

The six motion states are

$$z_{\mathrm{motion}}=[X,Y,\psi,v_x,v_y,r]^\mathsf{T}.$$

X and Y are global CG coordinates, psi is heading, vx/vy are body-frame velocities and r is yaw rate. Body x points forward and y left; positive steering and yaw are left turns. Let the front/rear CG distances be $l_d,l_t$, wheelbase $L=l_d+l_t$, and contact coordinates $(x_i,y_i)$ relative to the CG. Wheel order is **DI, DD, TI, TD**: front left, front right, rear left, rear right.

The chassis equations are

$$m\dot v_x=\sum_i F^c_{x,i}-R_a-\mathrm{sgn}(v_x)R_r-R_g+mv_y r,$$

$$m\dot v_y=\sum_i F^c_{y,i}-mv_xr,\qquad I_z\dot r=\sum_i(x_iF^c_{y,i}-y_iF^c_{x,i}).$$

Superscript c denotes body coordinates. The rotating frame produces the velocity-coupling terms; physical accelerometer acceleration is not identical to the derivative of a body velocity component. Position evolves as

$$\dot X=v_x\cos\psi-v_y\sin\psi,\quad\dot Y=v_x\sin\psi+v_y\cos\psi,\quad\dot\psi=r.$$

The resistance model includes aerodynamic drag, rolling resistance and road gradient:

$$R_a=\tfrac12\rho C_d A_fv_x|v_x|,\quad R_r=f_rmg\cos\theta,\quad R_g=mg\sin\theta.$$

The historical braking branch uses $v_x^2$ for drag within the forward-motion domain. Both branches clamp vx to nonnegative values. Reverse motion and extreme spins therefore require particular care; the clamp is an explicit model restriction, not a complete contact model.

The original scalar `Fx_sol` is a **force demand in newtons**, not total power. Initially it is allocated with fixed wheel weights. Power depends on force and speed; the later rotational formulation converts force demand to applied torque and calculates mechanical power using wheel speed.

## Four contact velocities and force transformations

Rigid-body kinematics give

$$v^c_{x,i}=v_x-r y_i,\qquad v^c_{y,i}=v_y+r x_i.$$

For wheel steering angle delta_i,

$$v^w_{x,i}=v^c_{x,i}\cos\delta_i+v^c_{y,i}\sin\delta_i,$$

$$v^w_{y,i}=-v^c_{x,i}\sin\delta_i+v^c_{y,i}\cos\delta_i.$$

Forces rotate back using the inverse transformation:

$$F^c_{x,i}=F_{x,i}\cos\delta_i-F_{y,i}\sin\delta_i,\quad F^c_{y,i}=F_{x,i}\sin\delta_i+F_{y,i}\cos\delta_i.$$

The consistent forward-motion slip-angle convention is either

$$\alpha_i=\delta_i-\mathrm{atan2}(v^c_{y,i},v^c_{x,i})$$

or, after rotating velocity to the wheel frame,

$$\alpha_i=-\mathrm{atan2}(v^w_{y,i},v^w_{x,i}).$$

One must not subtract the wheel-frame velocity angle from delta again. The traction branch regularizes its longitudinal denominator at low speed; this changes the definition near standstill and must not be confused with an exact rolling constraint.

## From independent limits to shared adhesion

V1 used linear lateral stiffness, $F^0_{y,i}=C_{\alpha,i}\alpha_i$, and independent longitudinal/lateral limits of magnitude $\mu_iF_{z,i}$. Both axes could then reach that magnitude simultaneously, giving a resultant $\sqrt2$ times the intended capacity.

V3 introduces the shared radius and radial scaling:

$$F_{\max,i}=\mu_iF_{z,i},\quad F^{\mathrm{req}}_{\mathrm{res},i}=\sqrt{(F^{\mathrm{req}}_{x,i})^2+(F^0_{y,i})^2},$$

$$\lambda_i=\min\left(1,\frac{F_{\max,i}}{\max(F^{\mathrm{req}}_{\mathrm{res},i},\epsilon)}\right),\quad F_{x,i}=\lambda_iF^{\mathrm{req}}_{x,i},\quad F_{y,i}=\lambda_iF^0_{y,i}.$$

This satisfies a common contact-force budget and preserves the direction of the requested resultant. It does not identify a tyre’s combined-slip force allocation. Saturation by itself also has no post-peak drop and does not prove that a wheel is mechanically locked.

Telemetry distinguishes **demanded force**, **transmitted force**, **nominal capacity** and **effective capacity**. Their utilization ratios answer different questions. Increasing a demand beyond capacity cannot by itself increase transmitted force.

## Load transfer and stiffness in V4

Static axle loads are

$$F_{z,d0}=\frac{mg l_t}{L},\qquad F_{z,t0}=\frac{mg l_d}{L}.$$

The quasi-static corrections use physical acceleration memories from the previous step:

$$\Delta F_{zx}=\frac{ma_{x,\mathrm{prev}}h_{cg}}L,$$

$$\Delta F_{zy,d}=\frac{k_{\mathrm{roll}}ma_{y,\mathrm{prev}}h_{cg}}{t_d},\qquad\Delta F_{zy,t}=\frac{(1-k_{\mathrm{roll}})ma_{y,\mathrm{prev}}h_{cg}}{t_t}.$$

For the four wheels,

$$F_z=F_{z0}+[-\Delta F_{zx}/2,-\Delta F_{zx}/2,\Delta F_{zx}/2,\Delta F_{zx}/2]+[-\Delta F_{zy,d},\Delta F_{zy,d},-\Delta F_{zy,t},\Delta F_{zy,t}].$$

The corrections sum to zero on the horizontal-road assumptions. Acceleration shifts load rearward; braking shifts it forward. Positive lateral acceleration loads the right wheels. `kRoll` distributes transfer between axles; it does not simulate springs, dampers or dynamic roll.

Stiffness varies sublinearly with load:

$$C_{\alpha,i}=C_{\alpha,i0}\left(F_{z,i}/F_{z,i0}\right)^{n_{Ca}},\qquad n_{Ca}=0.8.$$

The friction coefficient itself is not experimentally identified as load dependent. The selected traction branch rejects negative loads rather than pretending to simulate airborne contact. No dynamic suspension, pitch or roll state has been added.

## Empirical loss memory in V5 and its later rejection for longitudinal force

The early rear lateral-loss model measures excess demand rather than physical slip:

$$s_i=\max\left(0,\frac{|F^{\mathrm{req}}_{x,i}|}{\max(F_{\max,i},\epsilon)}-1\right),\qquad\eta_{\mathrm{target},i}=\frac1{1+K_{\mathrm{slip}}s_i}.$$

It applies during positive rear traction. A memory avoids instantaneous loss/recovery:

$$\eta_{k+1}=\eta_k+(1-e^{-dt/\tau})(\eta_{\mathrm{target}}-\eta_k).$$

The retained historical settings use 0.03 s for loss and 0.50 s for recovery. Rear lateral force is multiplied by eta after circle saturation. This allowed oversteer-like tests without wheel rotation, but it did **not** reduce longitudinal transmission in a straight-line start.

After modular TC was added, the absence of a clear uncontrolled launch penalty exposed that limitation. Multiplying Fx by the same eta was tested: one 2 s launch record gave 2.069 m/s without TC versus 17.188 m/s with TC. That numerical contrast did not establish a valid recovery law. A persistently excessive demand could keep an arbitrarily severe loss without representing wheel speed or torque balance, so the experiment was not selected as the final traction model.

The [characterization evidence](../Results/Modular_Integration/Traction_Characterization/README.md) and [rejected experiment](../Results/Modular_Integration/Discarded_Longitudinal_Eta/README.md) remain available.

## Historical braking loss and approximate locking

An acceleration-to-35 m/s then braking test motivated a distinction between normal saturation and reduced capacity during approximate lock. A nearly linear velocity segment alone is not unphysical: nearly constant net force produces nearly constant deceleration. Nor is g a universal braking limit; friction, loads and resistance determine the result.

The historical mode is triggered by **force demand**, not a physically integrated wheel stopping:

$$\rho_{b,i}=\frac{\max(0,-F^{\mathrm{req}}_{x,i})}{\max(\mu_iF_{z,i},\epsilon)}.$$

Entry uses 1.02, exit 0.90, with state retention between thresholds. The comparison uses nominal capacity to avoid self-triggering through the reduced limit. Full brake release exits the mode. In the assumed locked state, the speed-dependent target is

$$\eta_{\mathrm{lock}}(v)=\eta_{\min}+(1-\eta_{\min})e^{-|v^w_x|/v_b},\quad F^{\mathrm{eff}}_{\max,i}=\eta_{\mathrm{brake},i}\mu_iF_{z,i}.$$

The selected settings are $\eta_{\min}=0.68$ and $v_b=15$ m/s. Lower vb makes loss develop earlier with speed; lower eta_min deepens the high-speed reduction. This is a bounded phenomenological law, not a universal measured tyre relationship.

The same exponential memory update uses loss/recovery/release constants 0.03/0.10/0.01 s. On full release,

$$\eta(t)=1-(1-\eta_0)e^{-t/\tau_{\mathrm{release}}}.$$

At tau=0.01 s, about 95% of the deficit is recovered after 30 ms and 99.3% after 50 ms. This checks capacity-memory recovery, not mechanical spin-up of a real locked wheel.

The requested simplified lock mode sets lateral demand to zero, retaining a unit longitudinal factor on the already reduced capacity. It also uses zero force-transform steering angle in that mode, preventing residual `Fx*sin(delta)` steering. Real sliding friction can have a lateral dissipative component: setting all body lateral force to zero is a deliberate approximation, not an isotropic sliding-contact model.

## State and plant interface

Moving acceleration and adhesion memories into z made the plant reproducible through four formal inputs and two outputs:

```matlab
[z_next,tele] = pasoDinamicoSoloTraccion(z,u,p,dt);
```

| State entries | Meaning |
|---|---|
| 1–6 | X, Y, psi, vx, vy, r |
| 7–8 | Previous physical ax and ay for transfer |
| 9–12 | Historical lateral-loss memory slots |
| 13–16 | Historical braking-capacity memory |
| 17–20 | Approximate lock flags |
| 21–24 | Wheel angular-speed values |

V8 had twenty components; the rotational experiment extended it to twenty-four. The selected hybrid accepts either total force plus equivalent steering, or four wheel demands plus steering. Driving demands are restricted to the rear contacts. The current wrapper rejects simultaneous positive driving and negative braking commands across different wheels; selective braking while retaining motor torque is therefore not yet an admitted actuation mode.

`z_next` is the full next state, not only position. `tele` supplies forces, loads, capacities and diagnostics. Traction forces and accelerations are averaged over substeps; kappa/alpha are from the final substep. Braking returns NaN kappa because physical longitudinal slip is not computed in that branch. These differences matter when interpreting sensors and plots.

## Tracking, speed planning and stress tests in V9–V12

Pure Pursuit from practice 4 supplies a geometric driver without changing tyre physics. A curvature profile limits speed through

$$v_{\mathrm{curve}}=\min\left(v_{\max},\sqrt{a_{y,\max}/\max(|\kappa_{\mathrm{path}}|,\epsilon)}\right).$$

Forward/backward sweeps impose $v_{i+1}^2\le v_i^2+2a_{\mathrm{acc}}\Delta s$ and $v_i^2\le v_{i+1}^2+2a_{\mathrm{brake}}\Delta s$. A periodic profile query supports the closed track. Path curvature is distinct from longitudinal wheel slip despite sharing a common kappa notation in theory.

Pure Pursuit uses $\delta=\mathrm{atan2}(2L\sin\alpha_{\mathrm{target}},L_d)$ and $L_d=1+0.15\max(v_x,0)$ in the active configuration. It is not a yaw/side-slip stability controller. Reducing lookahead addressed premature corner cutting, but did not add ESC.

V10 expanded the track to fourteen corners, including a chicane, a 90-degree R4 m corner after a 172 m straight, and a 70-degree R3.5 m corner. Rapid exponential force application exposes limits without an ideal step. The 95% application time is approximately three time constants.

V11 raised planned lateral acceleration to 11 m/s², planned braking to 12 m/s² and removed extra anticipation. In its 35 m/s comparison, braking began 25.5 m later and lap time changed from 51.384 to 47.680 s. These planning bounds are not a guarantee of simultaneous combined-force feasibility.

V12 doubled maximum drive demand from 2500 to 5000 N and reduced acceleration time constant from 0.030 to 0.010 s. Its lap deteriorated to 149.036 s with 4.347 m maximum path error. This was an intentionally unstable stress case, not a performance improvement. The 3D scene remained planar physics with visual body volume.

## Longitudinal Magic Formula and wheel rotation

Physical driving slip requires the difference between peripheral and local forward velocities:

$$\kappa_i=\frac{R\omega_i-v^w_{x,i}}{\max(|v^w_{x,i}|,v_{\mathrm{reg}})}.$$

It can exceed one during traction; it is not clipped to a notional 100%. With sufficient forward speed a locked wheel has kappa about -1 under this convention. The 0.5 m/s denominator regularization changes the definition near standstill and prevents singular division; it is not complete zero-speed contact physics.

The pure longitudinal law is

$$F_x=D\sin\left[C\arctan\{B\kappa-E(B\kappa-\arctan(B\kappa))\}\right],\qquad D=\mu F_z.$$

The selected illustrative shape uses B=10, C=1.9, E=0.97. Its small-slip slope is BCD. These values are not identified from the project vehicle’s tyre; the [MathWorks reference](https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html) provides a typical dry-tarmac example of this form.

Force demand becomes an applied torque, $T_{\mathrm{req}}=RF^{\mathrm{req}}_x$, while wheel rotation follows

$$I_w\dot\omega=T_{\mathrm{motor}}-T_{\mathrm{brake}}-RF_x.$$

Excess torque changes wheel speed, slip and transmitted force. Recovery follows this coupled evolution and vehicle speed; it is not forced merely by elapsed time. Requested motor power is $\sum_iT_i\omega_i$; when it exceeds 60 kW the positive torques are scaled together. A constant high-force request is therefore not a constant delivered power.

Assumed wheel radius/inertia are 0.23 m and 0.60 kg·m². Traction wheel/chassis integration uses substeps up to 0.2 ms within a 2 ms outer step. Numerical checks covered symmetry, post-peak behaviour, energy balance and step refinement. None identifies a real tyre.

## Combined-slip experiment and rejection

Pacejka V2 added a pure lateral curve with $D_y=\mu_yF_z$, $C_y=1.3$, $E_y=-0.5$ and $B_y=C_\alpha/(C_yD_y)$ to preserve initial stiffness. It did not simply copy the longitudinal coefficient set to lateral force.

The experimental combined laws were $F_x=G_{x\alpha}F_x^0$ and $F_y=G_{y\kappa}F_y^0$, using

$$G_{x\alpha}=\frac1{\sqrt{1+(10\alpha)^2}},\qquad G_{y\kappa}=\frac1{\sqrt{1+(10\kappa)^2}}.$$

A final protective ellipse restricted shared capacity. The combined coefficient 10 was assumed, not fitted. At kappa=0.10, Gy is 0.707; at 0.30 it is 0.316. At 5 degrees of alpha, Gx is about 0.753. The old force limiters still referred to pure capacity and did not regulate physical slip, so a nominally limited torque request could exceed the new combined limit.

Both V2 circuit cases abandoned: 12.146 s without ABS/TC and 53.828 s with them. These are abandonment times, not lap times. The [eight-case audit](../Results/Tire_Model_Study/README.md) isolated the changes:

| Controlled variant | Time s | Maximum error m | Outcome |
|---|---:|---:|---|
| V1 reference | 47.726 | 0.343 | Complete |
| Pure lateral MF with V1 force priority | 47.708 | 0.390 | Complete |
| V2 without combined G factors | 47.488 | 0.239 | Complete |
| V2 with only Gxalpha | 47.374 | 0.269 | Complete |
| V2 with only Gykappa | 47.720 | 0.432 | Complete |
| Full V2, combined coefficient 10 | 53.828 | 10.001 | Abandon |
| Full V2, coefficient 3 sensitivity test | 47.552 | 0.290 | Complete |
| Full V2, coefficient 10 and lower demands | 11.064 | 10.001 | Abandon |

The results do not establish that lateral Magic Formula is unsuitable. They support an interaction between assumed coupling, unchanged force-based control and aggressive references. Lowering demands without updating the braking plan can also fail. Coefficient 3 is a sensitivity experiment, not a recommended calibration.

Fiala/brush, Dugoff, Burckhardt and dynamic-friction/relaxation models were considered as alternative levels of complexity. No alternative was silently inserted into the active plant. Each needs justified parameters and a stated domain; a familiar model name does not produce measured tyre data.

## Selected hybrid formulation

The explicit selection was **Pacejka only for rear driving slip, with historical braking restored**, rather than an exact return to V1, which also used rotational braking.

| Feature | Nonnegative total demand branch | Negative total demand branch |
|---|---|---|
| Rear longitudinal force | Pacejka with dynamic omega | Historical force demand and braking-capacity memory |
| Front wheel rotation | Kinematic rolling | Diagnostic reconstruction |
| Lateral force | Load stiffness with smooth remaining-circle saturation | Historical stiffness and radial circle scaling |
| Old eta multipliers | Reset to one | Historical braking loss/lock rule |
| Longitudinal slip output | Final-substep kappa | NaN |

The selected traction lateral law is

$$F_{y,\mathrm{cap}}=\sqrt{\max((\mu F_z)^2-F_x^2,0)},\qquad F_y=F_{y,\mathrm{cap}}\tanh\left(\frac{C_\alpha\alpha}{\max(F_{y,\mathrm{cap}},\epsilon)}\right).$$

It gives longitudinal force priority and no separate Gy(kappa) weighting. A post-peak reduction of Fx can therefore restore circle margin; this is not a fully identified combined-slip tyre. Rear omega is dynamic only during traction. Historical braking reconstructs all four omega values and sets the approximate-locked contacts to zero. Switching branches does not preserve a mechanically complete wheel-energy state.

Before Ackermann, 250 braking steps were checked against the restored historical branch, and changing Pacejka parameters did not change braking. A saved launch test gave 11.551 m/s without TC versus 21.997 m/s with TC after 3 s. These values are configuration-specific.

## Ackermann and the slip-angle correction

Parallel front angles were replaced by ideal Ackermann. Delta remains the virtual front bicycle angle, not a steering-wheel angle with an identified ratio. With y positive left,

$$\delta_i=\mathrm{atan2}(L\sin\delta,L\cos\delta-y_i\sin\delta),\qquad i\in\{DI,DD\}.$$

The rear angles remain zero. Front wheels share the instantaneous turning centre on the extension of the rear axle. In a positive turn, the inside DI angle is larger. For L=1.53 m, track=1.20 m and delta=0.2 rad, the front angles are approximately 0.2168/0.1856 rad. The 0.5 rad controller limit applies to the virtual angle; no individual mechanical rack limit was invented.

The historical braking branch had rotated velocity to the wheel frame and then used `delta-atan2(vyLocal,vxLocal)`, counting steering twice. It was changed to the consistent local expression given above. Traction already used that convention. This geometric correction is declared separately from the tyre law; it is not credited to Pacejka.

The [401-angle checks](../Results/Ackermann/README.md) verify symmetry, common radius, larger inside angle and identical straight-line response. Maximum common-radius error was approximately 2.84e-14 m and kinematically compatible residual lateral velocity 1.33e-15 m/s. Ackermann does not force zero slip during a genuinely dynamic manoeuvre. Disabling Ackermann retains the corrected slip-angle convention; it is not a full rollback of the earlier model.

## Sensors and seven-state estimation

The sensor block now provides longitudinal/lateral acceleration, yaw rate, intermittent GPS position and equivalent steering angle. Gaussian white noise and optional constant biases are separate from truth. `acc_x` aliases the existing `acc` sample, rather than adding a second independent measurement. GPS is 20 Hz at the 2 ms integration step; IMU and steering are sampled each step. Missing GPS/direction is represented by validity flags and NaN, not fabricated zeros.

Assumed standard deviations are 0.2 m/s² for ax/ay, 0.01 rad/s for gyro, 0.5 m per GPS axis and 0.1 degree for steering. The planar IMU is aligned at the CG on a horizontal road. Gravity projection, mounting offset, quantization and latency are not modelled. Forces, friction, eta memories and derived slip are not exposed as sensor measurements. Wheel speeds remain excluded because of their branch-dependent diagnostic construction.

The EKF state is

$$\hat x=[X,Y,\psi,v_x,v_y,a_y,\delta]^\mathsf{T}.$$

It preserves IMU-based prediction and intermittent GPS correction, adding lateral motion consistently:

$$\dot v_x=a_x+r v_y,\qquad\dot v_y=a_y-rv_x.$$

The new ay and delta measurements receive Kalman corrections with process uncertainty. Steering is filtered independently rather than imposing $r=v_x\tan\delta/L$, which would be inappropriate during drift. Longitudinal acceleration and yaw rate remain measured inputs; no new bias estimator or gyro filter is claimed. `dt=0` assimilates the initial sample without advancing time. The filter never reads true plant states or the actual injected sensor errors to correct them.

## External loop closure and force control

TC acts only on positive rear requests, limiting each to $0.98F_{\max,i}$; the coordinator then applies the smaller corrected value equally to both rear wheels. The earlier instantaneous ABS acted only on negative demands and used $-\min(|F^{\mathrm{req}}_{x,i}|,F_{\max,i})$. The current ABS instead modulates its factor from 0.98 toward 0.80 with release/hold/rebuild memory. Both generations are model-based force controls, not slip regulators, and do not independently request steering correction.

The existing TTC emergency branch has priority and bypasses the ordinary ABS/TC branches as originally specified. Its angle is the nominal steering angle, not an evasive-trajectory law. That priority is preserved, not endorsed as the final safety architecture. The normal saved circuit has no obstacle configured; emergency priority was checked separately at the interface level.

To preserve all thirteen existing block functions, [external capacity prediction](../Current%20State/estimarCapacidadControl.m) computes loads and braking-capacity memory from estimated signals, nominal parameters and previous applied commands. The [adapter](../Current%20State/adaptarEntradaControl.m) supplies the legacy twenty-slot input without reading physical tyre memories. Those controller-owned memories are deterministic calculations, not sensors, and receive no direct random noise. The spare lateral-memory slots are reserved constants; they are not estimates of true eta_slip.

Both tracking and speed-profile lookup use estimated position. True state is retained only for sensing, the declared known initial condition and evaluation. An explicitly separate perfect-state reference checks integration without becoming an invisible fallback.

## Retained earlier results and unresolved limits

The complete-loop controlled run completes in 47.370 s with 0.350 m maximum path error. The uncontrolled run completes in 101.228 s with 7.258 m maximum error; completion does not imply lane or cone clearance. The perfect-state controlled reference reproduces 47.132 s and 0.329 m. [Recorded histories and videos](../Results/Closed_Loop/README.md) retain all three cases and the failed first attempt.

Residual Q was manually adjusted in configuration after the inherited values failed the closed loop. A same-measurement replay reduced position RMSE from 0.372 to 0.142 m. This is provisional tuning, not a formal covariance identification. The full [Q analysis](Estimator_Q_and_Failure_Analysis.md) separates estimator improvement from vehicle stability.

Without TC, the first 1.5 m straight-line departure occurs at 5.634 s while position/vx estimation errors are only 0.145 m/0.026 m/s. Large estimator divergence develops afterwards. At 6.008 s the plant reaches its vx=0 clamp, absent from the EKF predictor. Small estimation perturbations can still trigger a fragile controller/plant interaction; these records do not validate uncontrolled extreme motion or absolve the estimator of robustness requirements.

The frozen current physics omits suspension dynamics, pitch/roll, camber, thermal/pressure effects, tyre relaxation and self-aligning moment. Parameters are illustrative. The mixed-command restriction, hybrid braking approximation and plant/estimator constraint mismatch remain explicit review items. New physical or estimator-internal changes require review before implementation. More complex equations, a smoother animation or one completed lap are not substitutes for experimental identification and independent validation.

## Source and reference map

- [Current source and configuration](../Current%20State/README.md), with Spanish MATLAB identifiers/comments retained.
- [V1–V12 history](Versions/README.md) and [post-V12 chronology](Versions/After_V12/README.md).
- [Archived alternatives](../Dynamic%20Model/Archived%20Models/README.md), [modular results](../Results/Modular_Integration/README.md) and [slip-model audit](../Results/Tire_Model_Study/README.md).
- [Publication artifact hashes](closed_loop_artifact_manifest.json), distinguishing original recorded evidence from current source.
- [MathWorks longitudinal Magic Formula](https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html), [steering geometry](https://www.mathworks.com/help/vdynblks/ref/steeringsystem.html) and [EKF algorithms](https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html).


## Latest 33-corner integration and defensible conclusion

The fixed physical model now runs with sensing, seven-state estimation and ABS/TC/ESC in the dedicated [33-corner harness](../Current%20State/Ensayos_circuito_extremo/README.md). The full loop completes in 191.074 s versus 190.006 s with perfect state; maximum actual path error is 0.504636 m and position-estimator RMSE is 0.153999 m. Sensor noise, Q and the core block equations are unchanged. [Evidence and design interpretation](Design_Decisions/Extreme_33_Corners_2026-11-28.md).

The model qualitatively reproduces the main adhesion-loss phenomena and coordinated driving-aid action, with physically plausible results within its assumptions. Quantitative agreement with a particular vehicle would require identification and experimental validation. That distinction does not make experimental tyre identification or production slip regulators required unfinished work: force-comparison ABS/TC are the stated academic scope. Further evidence should target robustness, individual aids, emergency scenarios and report reproducibility.
