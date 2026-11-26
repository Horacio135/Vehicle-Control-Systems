# Simulation video gallery

[Watch the gallery](https://Horacio135.github.io/Vehicle-Control-Systems/).

The static gallery features nine recordings and retains ten additional recordings as archive links. Current estimated-loop results, diagnostics and historical milestones are labelled separately. Source videos remain in their existing repository locations.

## Maintenance and deployment

Edit `gallery/index.html` and keep its embedded catalogue synchronized with `gallery/catalog.json`. Run `python gallery/build_gallery.py` from the repository root to build `_site`. The builder validates the media inventory and copies only selected videos and posters. GitHub Actions deploys that artifact on pushes to main affecting the gallery, results or workflow. Playback is requested only after clicking Play; videos are not downloaded on initial page load.

## Included recordings

| Section | Title | Repository file | Reason |
|---|---|---|---|
| current | Actual versus estimated trajectory | `PROJECT/Results/Closed_Loop/Actual_vs_estimated_ABS_TC_2D.mp4` | The clearest current demonstration: complete sensing/estimation loop, ABS/TC on, normal speed and error plots. |
| current | ABS and TC on versus off | `PROJECT/Results/Closed_Loop/Closed_loop_ABS_TC_comparison_2D_x4.mp4` | Direct comparison of the current estimated loop with both force limiters enabled or disabled. |
| diagnostic | First failed complete-loop attempt | `PROJECT/Results/Closed_Loop/First_Failed_Attempt/First_failed_attempt_2D.mp4` | Retain as diagnostic evidence for the inherited-Q failure. Explicitly not the current tuned result. |
| history | V9 curvature-based speed planning | `PROJECT/Results/Hybrid_Tests/Seguimiento_V9_planificador_5_velocidades.mp4` | Shows the planning milestone before later tyre and estimator changes. |
| history | V10 fourteen-corner stress circuit | `PROJECT/Results/Hybrid_Tests/Seguimiento_V10_15_20_25_30_35_20261117.mp4` | Choose the higher-cap campaign because its 35 m/s case provides the more useful comparison to later stages. |
| history | V11 single-car 3D view | `PROJECT/Results/Hybrid_Tests/Seguimiento_V11_35ms_3D.mp4` | Retain one clear 3D presentation milestone; it is not the current estimated loop. |
| history | V12 aggressive acceleration in 3D | `PROJECT/Results/Hybrid_Tests/Seguimiento_V12_35ms_3D_Aceleracion_agresiva.mp4` | Explains the decision to develop traction/stability control by showing an intentionally severe pre-Pacejka test. |
| history | Longitudinal Pacejka V1 experiment | `PROJECT/Results/Modular_Integration/Pacejka_V1/Pacejka_V1_ABS_TC_2D.mp4` | Shows the rotational force–slip milestone. This branch also used rotational braking and differs from the selected hybrid. |
| history | Discarded combined-slip V2 experiment | `PROJECT/Results/Modular_Integration/Discarded_Pacejka_V2/Discarded_Pacejka_V2_ABS_TC_2D.mp4` | Preserve the failed alternative and its design rationale; do not hide rejection or label it as the current model. |

## Not embedded in the gallery

These videos are not deleted. Their direct links remain available in the archive.

| Title | Repository file | Reason |
|---|---|---|
| V8 initial hybrid tracking | `PROJECT/Results/Hybrid_Tests/Seguimiento_hibrido_animado.mp4` | Superseded by V9 planning and the current complete-loop demonstrations. |
| V10 lower-cap stress campaign | `PROJECT/Results/Hybrid_Tests/Seguimiento_V10_Estres_14_curvas.mp4` | Overlaps the selected V10 campaign; keep in the repository archive rather than duplicating the gallery. |
| V11 multiple-cap demanding map | `PROJECT/Results/Hybrid_Tests/Seguimiento_V11_Mapeo_limite.mp4` | Same planning milestone as the selected V11 view; retain detailed speed-cap analysis in its README. |
| Initial modular ABS/TC comparison | `PROJECT/Results/Modular_Integration/Initial/ABS_TC_comparison_2D.mp4` | Earlier physics and no current EKF; the current on/off comparison is the primary presentation. |
| Initial modular realtime 2D | `PROJECT/Results/Modular_Integration/Initial/ABS_TC_realtime_2D.mp4` | An older single-case presentation; superseded by the current real-versus-estimated video. |
| Initial modular realtime 3D | `PROJECT/Results/Modular_Integration/Initial/ABS_TC_realtime_3D.mp4` | Pre-Pacejka, pre-Ackermann and pre-EKF. Omit to avoid confusing it with a current 3D full-loop result. |
| Pure Pursuit adjustment comparison | `PROJECT/Results/Modular_Integration/PurePursuit_Adjustment/ABS_TC_comparison_2D.mp4` | Useful specialist tracking evidence, but redundant for a compact public gallery. Preserve its README and direct archive link. |
| V5 skidpad drift | `PROJECT/Results/V5_Slip_Tests/Animacion_derrape_skidpad.mp4` | Early phenomenological loss model, superseded by later tyre experiments and the current hybrid. |
| V5 slalom | `PROJECT/Results/V5_Slip_Tests/Animacion_Eslalon.mp4` | Early manoeuvre footage; keep available in the archive without mixing it with current validation. |
| V5 drift demonstration | `PROJECT/Results/V5_Slip_Tests/Derrape.mp4` | Short early illustration already represented by later instability experiments. |

## Size and review

The catalogue covers 19 published MP4s. The 9 selected recordings total approximately 117.0 MiB; poster sizes are additional. This is total stored media, not an initial page download. Playback requires fetching the selected video. Test mobile layout, playback, file paths and the Pages URL before publishing. The current results are a single saved simulation, not experimental validation.
