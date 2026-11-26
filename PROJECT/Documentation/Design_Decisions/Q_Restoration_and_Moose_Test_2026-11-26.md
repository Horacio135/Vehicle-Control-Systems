# Q restoration and retained Moose Test 02

[← Estimator analysis](../Estimator_Q_and_Failure_Analysis.md) · [Moose Test 02 evidence](../../Results/Moose_Test_02/README.md) · [Runnable scenario](../../Current%20State/Ensayos_alce_seguimiento/)

## Restore uncertainty assumptions before further estimator work

On 26 November 2026, the original residual process covariance for `[X,Y,psi,vx,vy]` was restored:

```matlab
p.Q = diag([.02 .02 .005 .10 .10].^2);
```

The temporary `diag([.001 .001 .00001 .003 .003].^2)` setting is no longer the active configuration. It had improved a same-measurement replay and enabled the previously recorded controlled lap, but that was a scenario-specific change in the filter's assumed process uncertainty, not a reduction of simulated sensor noise or a validated general estimator redesign.

Sensor noise, R, initial P, the EKF equations, `q_ay` and `q_direccion` remain unchanged. The initial five-state covariance is `eye(5)*1e-3`; the seven-state estimator appends ay and steering variances before assimilating measurements. Future P0 work must reflect actual initial-state uncertainty and distinguish initialization effects from persistent estimation errors. No P0 adjustment has been implemented.

**No new complete-loop simulation has been performed after restoring Q.** Existing full-loop MP4s, replay CSVs and lap/error metrics remain labelled as historical reduced-Q results. They must not be presented as validation of the active restored-Q package.

The next diagnostic priority is the straight-line spin: compare steering requested from estimated versus true states on the same recorded vehicle trajectory, together with force demand and rear lateral-capacity margin. Small estimation errors can initiate harmful corrections near the plant's limit, while large errors after a spin do not prove that estimator divergence caused the initial event. No unreviewed estimator-internal fix was applied.

## Separate lateral-instability exploration from estimator evaluation

The initial exploration contained four open-loop evasive manoeuvres and four sinusoidal steering cases. A later four-case campaign used the existing Pure Pursuit module and a constant speed reference. These scenarios omitted sensors, the estimator and ESC so that lateral behaviour could be examined separately.

The user subsequently requested direct deletion of all eight initial cases and all later cases except **Alce_PP_02**. Only that retained case, its source scenario and its results are published in this update. Removed campaigns are not reconstructed or duplicated in the archive or gallery.

## Retained configuration

| Setting | Value |
|---|---:|
| Initial speed and fixed speed reference | 20 m/s |
| Lateral displacement | 4 m |
| Length of each lane-change transition | 10 m |
| Straight offset section between transitions | 10 m |
| First transition starts at | X = 40 m |
| Steering | Existing Pure Pursuit module |
| Longitudinal demand | Existing PI maintenance mode |
| ABS / TC | Enabled |
| Sensors / estimator / ESC | Omitted |

The steering and longitudinal modules receive the real simulated state. For longitudinal regulation, the scalar speed input is explicitly adapted to actual speed magnitude `hypot(vx,vy)`; Pure Pursuit still receives body longitudinal velocity. The scenario sets `p.conductor.errorActivar=Inf` to use the existing PI maintenance mode with its original gains, rather than repeatedly requesting full motor force for small speed deviations. This is scenario configuration, not a block-internal modification.

Negative longitudinal requests are clipped to zero to preserve the no-braking condition. The reference is fixed but actual speed is not imposed on the plant; exact constant-speed tracking cannot be claimed during loss of control. Rear drive requests remain equal and TC can reduce them. ABS cannot intervene without a braking request. The obstacle is visual and no impact force is modelled. This is an exploratory evasive manoeuvre, not an ISO moose-test certification.

## Recorded outcome and interpretation

| Metric | Recorded value |
|---|---:|
| Recorded simulation time | 3.704 s |
| Maximum speed-magnitude error | 5.099 m/s |
| Speed-magnitude RMSE | 1.182 m/s |
| Maximum requested steering angle | 28.648 deg |
| Maximum vehicle sideslip magnitude | 87.954 deg |
| Maximum rear slip-angle magnitude | 88.362 deg |
| Maximum rear force-circle utilization | approximately 100% |
| TC-limited samples | 352 at a 2 ms step |
| ABS-limited samples | 0 |
| Visual rectangle contact with obstacle | none recorded |

The run loses control and is terminated when body longitudinal velocity drops below 0.5 m/s. It is **not a successful complete manoeuvre** and does not maintain speed exactly. The very large slip angles are diagnostic evidence at the model's extreme operating range, not experimentally validated tyre behaviour. Absence of rectangle contact before termination does not establish that the whole obstacle-avoidance course was completed.

The tyre model, Ackermann geometry and ABS/TC block implementations were unchanged for this scenario. Recorded block hashes describe the source at simulation time; a later configuration change must not be misrepresented as a hash failure or as a rerun of this evidence.
