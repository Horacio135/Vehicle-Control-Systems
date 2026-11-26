# Longitudinal and lateral slip-model audit

Eight controlled ablations isolate the interaction of combined-slip weights, force limiters and aggressive references. The standalone 53 MB audit MAT is omitted; the summary CSV, figures, theory and model source retain the findings.

[← Results](../README.md)

## Recorded artifacts

- [ablation_study.csv](ablation_study.csv)
- [ablation_study.png](ablation_study.png)
- [discarded_v2_first_corner.png](discarded_v2_first_corner.png)
- [discarded_v2_timeline.csv](discarded_v2_timeline.csv)

## Causal finding

Pure lateral MF in the V1 force-priority formulation completes in 47.708 s with 0.390 m maximum error. V2 with both combined weights at 10 abandons at 53.828 s. Removing either weight restores completion in that campaign. This does not justify removing physical coupling; it demonstrates that the assumed coupling was not calibrated. See [the theory](../../Documentation/Dynamics_Development.md#combined-slip-experiment-and-rejection).
