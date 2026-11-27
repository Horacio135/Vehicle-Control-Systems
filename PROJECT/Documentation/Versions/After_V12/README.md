# Development after V12

[← Version history](../README.md) · [Theory](../../Dynamics_Development.md) · [Current state](../../../Current%20State/README.md)

These are named integration milestones, not invented V13–V20 releases. V1–V12 retain their original chronology.

| Milestone | Decision and outcome | Evidence |
|---|---|---|
| Block extraction and force control | Separate functions; add force-based ABS and rear TC, emergency priority and equal rear motor force | [Modular results](../../../Results/Modular_Integration/README.md) |
| Pure Pursuit refinement | Shorter lookahead reduces premature corner cutting; does not add stability control | [Tracking adjustment](../../../Results/Modular_Integration/PurePursuit_Adjustment/README.md) |
| Longitudinal eta experiment | Applying old lateral-loss eta to Fx exposed excessive persistent loss; rejected as final law | [Experiment](../../../Results/Modular_Integration/Discarded_Longitudinal_Eta/README.md) |
| Pacejka V1 | Add wheel rotation and longitudinal force–slip law, initially including braking | [V1 results](../../../Results/Modular_Integration/Pacejka_V1/README.md) |
| Combined Pacejka V2 | Lateral curve and uncalibrated combined-slip weights fail the circuit; version discarded | [V2 results](../../../Results/Modular_Integration/Discarded_Pacejka_V2/README.md) |
| Traction-only selection | Keep Pacejka only for rear driving slip; recover historical braking | [Selected branch](../../../Results/Modular_Integration/Traction_Only/README.md) |
| Slip-model audit | Eight ablations show that pure lateral Magic Formula itself is not the demonstrated cause of V2 failure | [Study](../../../Results/Tire_Model_Study/README.md) |
| Ackermann, 23 Nov | Individual front angles and corrected historical slip-angle geometry; 47.132 s controlled lap | [Geometry checks](../../../Results/Ackermann/README.md) |
| Sensors, 26 Nov | Add lateral acceleration and equivalent steering measurement; 20,000-sample checks | [Sensor checks](../../../Results/Sensors/README.md) |
| EKF, 26 Nov | Extend to seven states; preserve prediction/GPS-correction method | [Estimator checks](../../../Results/Estimator/README.md) |
| Full loop, 26 Nov | External adapters keep all thirteen block functions unchanged; tune residual Q; 47.370 s controlled lap | [Full-loop results](../../../Results/Closed_Loop/README.md) |

| Modulated ABS, 27 Nov | Independent release/hold/rebuild; TC on in both comparison cases; 47.294 s controlled lap | [New evidence](../../../Results/Modulated_ABS/README.md) |

Physical limits and rejected alternatives are retained explicitly. Current full-loop videos must not be relabelled as evidence for earlier tyre models, and earlier 3D videos do not imply a new 3D estimated-loop run.
