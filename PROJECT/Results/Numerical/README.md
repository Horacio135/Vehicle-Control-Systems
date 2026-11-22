# Numerical evidence

[← Results](../README.md) · [All-run summary](summary_all_runs.csv) · [Video inventory](video_inventory.csv) · [Run sources](../Run_Sources/README.md)

**50 recorded histories**, exported from previously saved MAT files without rerunning simulations or modifying the plant. A history is a recorded execution: repeated baseline cases, kinematic/dynamic manoeuvres and braking tests are included. These are not 50 independent experiments.

## Files and sampling

| Files | Contents |
|---|---|
| [summary_all_runs.csv](summary_all_runs.csv) | Metrics calculated over **all original samples** |
| [video_inventory.csv](video_inventory.csv) | Duration, fps and dimensions checked with VideoReader |
| `V*_case*.csv` / `V*_hist*.csv` | Temporal state/telemetry, selected at approximately **20 Hz**, always including the last sample |
| `V*_config.json` | Available vehicle, planner and test parameters; missing parameters are not borrowed from another campaign |
| [raw_evidence_manifest.json](raw_evidence_manifest.json) | Size and SHA256 of source MAT files, not duplicated here |

Original integration uses approximately 0.002 s steps. Temporal CSVs are reduced for inspection. Do not use their sampling to estimate 10–30 ms peaks: summary extrema and percentages are calculated **before** downsampling. Large, partly duplicate raw MAT files remain in the local evidence archive; archived MLXs, snapshots and parameters allow campaigns to be regenerated.

## Columns and units

`t_s`, `X_m`, `Y_m`, `psi_rad`, `vx_ms`, `vy_ms`, `r_rads` and z-memory fields. `Fx_sol` and `Fxw/Fyw/Fzw/Fmax_*` use N; `delta` uses rad; `ax/ay` use m/s². `eta_*`, `lambda_w` and utilization are dimensionless. Four-wheel suffixes follow DI, DD, TI, TD: front left, front right, rear left, rear right. `vref` is the controller reference, not actual speed.

`peak_vx_ms` is peak longitudinal speed; `peak_speed_ms` uses hypot(vx,vy). Maximum/RMS lateral error uses ect. `CG_outside_pct` follows the recorded CG criterion. `lock_pct` counts samples with any wheel in lock mode; `saturated_pct` counts any lambda<1-1e-6. Neither is a physical slip ratio.

`complete=1` means lap completion where `completa` exists, or reaching the stop criterion in tests recording `paradaAlcanzada`. `complete=0` means a recorded criterion was not reached. **NaN means not recorded**, not failure. Older manoeuvres do not always have a completion criterion comparable with the hybrid circuit. `stop_reason` records the available reason, translated into English without changing numeric data.

## Campaigns

| Dataset | Configuration |
|---|---|
| V08_standard | Skidpad, slalom, standstill runs and straight-line test; interface comparison |
| V08_braking_turn | 2°/10° steering after 4 m of braking |
| V08_hybrid | Initial 6/14 m/s test; 14 m/s loses tracking |
| V09_four_caps | 14, 18, 22, 26 m/s |
| V09_five_caps | 6, 10, 14, 18, 22 m/s |
| V10_five_caps | 6, 10, 14, 18, 22 m/s |
| V10_current_caps | 15, 20, 25, 30, 35 m/s |
| V11_initial_comparison | 15, 20, 25, 30, 35 m/s; baseline and demanding map |
| V11_current_caps | 25, 30, 32, 35, 40 m/s; saved demanding-map histories |
| V11_single_3D | One car, 35 m/s cap |
| V12_aggressive_3D | One 35 m/s extreme-demand car and its baseline |

The first saved hybrid test, at 6/14, differs from the current V8 file, at 6/10. That difference is preserved: the failure at 14 m/s is not attributed to the 10 m/s case. Figures and videos have their own campaign configurations in their README files.
