# Whole-track ESC comparison with temporary driver recovery

[Campaign overview](../README.md) · [Top-down video, x8](Cenital_completa_ABS_TC_con_y_sin_ESC_x8.mp4)

ABS and TC are enabled in both perfect-state cases. Only ESC differs. The orange no-ESC car completes in 806.824 s with 11 recovery episodes and 5,808.05 m travelled; the blue ESC car completes in 190.006 s with none. These are not equivalent-demand lap-performance scores because recovery temporarily changes the driver's actions and the no-ESC car leaves the course repeatedly. Cones are not collision obstacles.

Recovery triggers after 30 m when sideslip exceeds 25 degrees or centreline error exceeds 2.5 m. The driver first requests stopping with neutral steering while total speed exceeds 3 m/s or yaw rate exceeds 0.3 rad/s; it then requests 2 m/s and steers toward the track. Recovery ends after 0.5 s with lateral error below 0.6 m, heading error below 0.2 rad and sideslip below 0.1 rad. **The original aggressive speed profile resumes after every recovery; no persistent prudent-speed cap is retained.** The recovery rule is available in both cases but only the no-ESC case uses it. It uses ordinary steering/longitudinal commands, not an extra selective-braking ESC. Neither pose nor tyre states are reset.

Completion requires the accumulated lap length, centreline error below 1 m and recovery finished. The no-ESC vehicle regains the lane slightly beyond the finish section. Recovery episodes are not necessarily complete high-speed spins. The video keeps the ESC car at its final recorded state while the other continues; its enlarged symbols and heading arrows aid whole-track visibility.

To keep each artifact below GitHub's file-size limit, histories are stored as `Caso_1.mat` and `Caso_2.mat`, and other fields of the original combined MAT are stored in `Contexto_comparacion.mat`. Both histories were checked with `isequaln` against the original combined file. The portable renderer reconstructs the same `historias` cell array; no simulation is rerun or numerical values changed by this split.
