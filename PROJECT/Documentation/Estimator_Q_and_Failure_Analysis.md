# Estimator uncertainty and straight-line departure analysis

[← Documentation](README.md) · [Current implementation](../Current%20State/README.md) · [Evidence](../Results/Closed_Loop/README.md)

The residual process covariance Q was adjusted to close the first sensing–estimation loop. This improved estimation on the recorded controlled trajectory, but it does not establish robustness during uncontrolled spins. This note separates measured facts, implementation limitations and the mechanism supported by the records.

## Meaning of Q in this implementation

The EKF represents uncertainty, rather than reversing the random errors added by the sensor simulator. Its motion covariance prediction is

$$P^- = FPF^\mathsf{T} + Q_{\mathrm{res}} + G\Sigma_{\mathrm{IMU}}G^\mathsf{T}.$$

Here F is the state-transition Jacobian, P is state-error covariance, and the G term explicitly propagates longitudinal accelerometer and gyro noise. Q represents additional prediction/model uncertainty. R is the GPS measurement covariance, unchanged at $0.5^2I_2$ in this run. Q changes the filter's weighting; it does not change actual sensor noise or inject random values into the physical model.

Q is configured for `[X,Y,psi,vx,vy]` at the 2 ms reference step. The seven-state estimator separately adds process uncertainty for ay and delta using `q_ay=25` and `q_direccion=0.002` per second. Those two values were not retuned for this closure.

```matlab
% Before: inherited residual uncertainty
p.Q = diag([.02 .02 .005 .10 .10].^2);
% Current configuration
p.Q = diag([.001 .001 .00001 .003 .003].^2);
```

The entries inside the vectors are squared to form variances; their units differ by state. Covariance is scaled by `dt/p.dt`. A smaller residual Q generally gives more weight to continuity of the IMU-based prediction and allows smaller GPS-driven changes. It is not universally better: model error and unestimated bias can then persist longer.

The inherited residual values were too permissive for this implementation, which already propagates IMU uncertainty explicitly. The first closed-loop attempt failed at 6.046 s. Q was manually reduced and the complete scenario was checked again. This is a **provisional, scenario-specific adjustment**, not maximum-likelihood identification, a parameter sweep or experimental calibration. No sensor-noise level, tyre coefficient, force limit, speed plan or Pure Pursuit gain was changed to obtain the tuned lap.

## Isolated comparison with the same measurements

Closed-loop retuning changes the vehicle trajectory, so comparing two different laps cannot isolate estimator quality. Both Q settings were therefore replayed on exactly the same saved measurements of the current controlled lap, with the same initial condition and unchanged EKF equations.

| RMS error | Previous Q | Current Q |
|---|---:|---:|
| Position norm | 0.371911 m | 0.142235 m |
| Heading | 2.634017 deg | 0.159064 deg |
| Longitudinal velocity | 0.609200 m/s | 0.047784 m/s |
| Lateral velocity | 0.669899 m/s | 0.043727 m/s |
| Maximum position error | 1.252328 m | 0.372671 m |

The replay with current Q reproduces the saved estimated state to within 1e-10. The [CSV](../Results/Closed_Loop/same_measurement_Q_replay.csv) retains the values. This is evidence of improvement on those measurements, not proof that current Q is optimal in every manoeuvre.

## What ABS and TC actually do

Neither limiter directly commands yaw correction. Pure Pursuit requests steering; no ESC yaw-feedback law has been added. TC limits positive rear drive demands and the coordinator applies the smaller corrected rear value equally to both driven wheels. ABS limits negative demands independently and reduces entry into the historical approximate-lock mode, which removes local steering authority.

Their effect on path following is indirect: they keep force demands in a region where lateral response is more manageable. The current force-circle implementation shares capacity between x and y. The first uncontrolled straight-line departure occurs while the driver still requests **+5000 N**, so TC is the relevant aid at that event; ABS cannot explain an improvement during positive traction.

At the first departure, rear longitudinal utilization is approximately 0.9997 and 0.9902. The remaining lateral force-circle margins computed from the recorded step are only about 17.9 N and 167.4 N. Small steering perturbations can therefore interact with a rear axle very near its longitudinal limit. This supports an instability mechanism in the model; it does not experimentally validate a real tyre or prove that every straight-line excursion is realistic.

## Chronology distinguishes initial instability from estimator divergence

| Event, current estimated loop without ABS/TC | Time or value |
|---|---:|
| First true path deviation beyond 1.5 m | 5.634 s |
| Position error of EKF at that event | 0.145 m |
| Longitudinal velocity error at that event | 0.026 m/s |
| Vehicle lateral velocity at that event | -4.218 m/s |
| Steering demand at that event | 7.302 deg |
| First longitudinal plant clamp after startup | 6.008 s |
| First vx estimation error beyond 1 m/s | 6.014 s |
| Position-estimation error at 7 s | 15.932 m |

The large position error therefore follows the initial departure. Small measurement/estimation perturbations can still trigger steering corrections and expose a fragile loop; this does not absolve the estimator or the controller from robustness requirements. In the earlier perfect-state uncontrolled reference the first 1.5 m departure is later, at 12.238 s and in a corner. Noise and closed-loop interaction materially change the outcome.

## A concrete model mismatch

The plant uses

$$v_{x,k+1}=\max(v_{x,k}+\dot v_x\,dt,0),$$

whereas the EKF predictor uses

$$\hat v_{x,k+1}=\hat v_{x,k}+(a_x+r\hat v_y)dt.$$

During severe rotation the plant may activate its no-reverse clamp while the estimator continues its unconstrained motion prediction. The two models then disagree even with accurate accelerations. Tightening Q cannot repair a missing constraint and can delay recovery from model error. IMU time averaging, nonlinear motion and unestimated sensor biases are further limitations to examine; they were not all isolated by this test.

## Pending review

Before editing any block internals, review the plant/estimator constraint mismatch, covariance consistency during extreme motion and whether available measurements make the desired states sufficiently observable. Separate TC-only and ABS-only trials, same-measurement replays, additional seeds and GPS outages are the next useful experiments. A good controlled lap alone must not be used as validation of the full operating range. No block was changed during this diagnostic analysis.

References: [MathWorks EKF algorithms](https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html), [current source](../Current%20State/Estimacion/estimadorEstado.m), [recorded results](../Results/Closed_Loop/README.md).
