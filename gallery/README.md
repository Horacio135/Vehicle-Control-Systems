# Simulation video gallery

[Watch the gallery](https://horacio135.github.io/Vehicle-Control-Systems/).

The gallery has 10 players and 10 archive links. The only latest steering result is moose-style test 02; the full-loop videos are labelled historical evidence with previous reduced Q. No complete-loop validation with restored Q has been published.

## Maintenance

Edit `gallery/catalog.json` and synchronize the embedded catalogue in `gallery/index.html`. Run `python gallery/build_gallery.py` from the repository root. GitHub Actions publishes selected media after a push to main. Videos are loaded only after Play is clicked.

## Catalogue

| Section | Recording | Reason |
|---|---|---|
| current | [Retained moose-style test 02 — failed manoeuvre](../PROJECT/Results/Moose_Test_02/Alce_PP_02_2D_capacidad.mp4) | Pure Pursuit with a fixed 20 m/s target, ABS/TC enabled, and no sensors, estimator or ESC. Actual speed is not held exactly; recording aborts after loss of control. Only this steering case is retained. |
| history | [Actual versus estimated trajectory — previous reduced Q](../PROJECT/Results/Closed_Loop/Actual_vs_estimated_ABS_TC_2D.mp4) | Historical full-loop evidence with reduced Q. Active Q has been restored; this is not a validation of that restored configuration. |
| history | [ABS and TC on versus off — previous reduced Q](../PROJECT/Results/Closed_Loop/Closed_loop_ABS_TC_comparison_2D_x4.mp4) | Historical full-loop evidence with reduced Q. Active Q has been restored; this is not a validation of that restored configuration. |
| diagnostic | [First failed complete-loop attempt](../PROJECT/Results/Closed_Loop/First_Failed_Attempt/First_failed_attempt_2D.mp4) | Historical failure using inherited Q before the temporary reduced-Q experiment. No new restored-Q complete-loop run has been published. |
| history | [V9 curvature-based speed planning](../PROJECT/Results/Hybrid_Tests/Seguimiento_V9_planificador_5_velocidades.mp4) | Shows the planning milestone before later tyre and estimator changes. |
| history | [V10 fourteen-corner stress circuit](../PROJECT/Results/Hybrid_Tests/Seguimiento_V10_15_20_25_30_35_20261117.mp4) | Choose the higher-cap campaign because its 35 m/s case provides the more useful comparison to later stages. |
| history | [V11 single-car 3D view](../PROJECT/Results/Hybrid_Tests/Seguimiento_V11_35ms_3D.mp4) | Retain one clear 3D presentation milestone; it is not the current estimated loop. |
| history | [V12 aggressive acceleration in 3D](../PROJECT/Results/Hybrid_Tests/Seguimiento_V12_35ms_3D_Aceleracion_agresiva.mp4) | Explains the decision to develop traction/stability control by showing an intentionally severe pre-Pacejka test. |
| history | [Longitudinal Pacejka V1 experiment](../PROJECT/Results/Modular_Integration/Pacejka_V1/Pacejka_V1_ABS_TC_2D.mp4) | Shows the rotational force–slip milestone. This branch also used rotational braking and differs from the selected hybrid. |
| history | [Discarded combined-slip V2 experiment](../PROJECT/Results/Modular_Integration/Discarded_Pacejka_V2/Discarded_Pacejka_V2_ABS_TC_2D.mp4) | Preserve the failed alternative and its design rationale; do not hide rejection or label it as the current model. |
| archive link | [V8 initial hybrid tracking](../PROJECT/Results/Hybrid_Tests/Seguimiento_hibrido_animado.mp4) | Superseded by V9 planning and the current complete-loop demonstrations. |
| archive link | [V10 lower-cap stress campaign](../PROJECT/Results/Hybrid_Tests/Seguimiento_V10_Estres_14_curvas.mp4) | Overlaps the selected V10 campaign; keep in the repository archive rather than duplicating the gallery. |
| archive link | [V11 multiple-cap demanding map](../PROJECT/Results/Hybrid_Tests/Seguimiento_V11_Mapeo_limite.mp4) | Same planning milestone as the selected V11 view; retain detailed speed-cap analysis in its README. |
| archive link | [Initial modular ABS/TC comparison](../PROJECT/Results/Modular_Integration/Initial/ABS_TC_comparison_2D.mp4) | Earlier physics and no current EKF; the current on/off comparison is the primary presentation. |
| archive link | [Initial modular realtime 2D](../PROJECT/Results/Modular_Integration/Initial/ABS_TC_realtime_2D.mp4) | An older single-case presentation; superseded by the current real-versus-estimated video. |
| archive link | [Initial modular realtime 3D](../PROJECT/Results/Modular_Integration/Initial/ABS_TC_realtime_3D.mp4) | Pre-Pacejka, pre-Ackermann and pre-EKF. Omit to avoid confusing it with a current 3D full-loop result. |
| archive link | [Pure Pursuit adjustment comparison](../PROJECT/Results/Modular_Integration/PurePursuit_Adjustment/ABS_TC_comparison_2D.mp4) | Useful specialist tracking evidence, but redundant for a compact public gallery. Preserve its README and direct archive link. |
| archive link | [V5 skidpad drift](../PROJECT/Results/V5_Slip_Tests/Animacion_derrape_skidpad.mp4) | Early phenomenological loss model, superseded by later tyre experiments and the current hybrid. |
| archive link | [V5 slalom](../PROJECT/Results/V5_Slip_Tests/Animacion_Eslalon.mp4) | Early manoeuvre footage; keep available in the archive without mixing it with current validation. |
| archive link | [V5 drift demonstration](../PROJECT/Results/V5_Slip_Tests/Derrape.mp4) | Short early illustration already represented by later instability experiments. |
