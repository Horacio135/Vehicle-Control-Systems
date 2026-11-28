# Temporary 33-corner complete-loop video

Temporary gallery access copy requested on 28 November 2026 because the mobile Codex file connection could not open the local MP4. This publication adds the recording and its preview only; it does not update or replace the repository's runnable model package.

The existing 4,908.2 m, 33-corner scenario completes in **191.074 s** with noisy sensors, the EKF and ABS/TC/ESC. Blue is the actual vehicle; the orange dotted outline uses estimated X, Y and yaw. It is not an uncertainty envelope or a second vehicle. Playback is normal speed, 20 fps.

- Maximum actual centreline error: 0.505 m.
- Euclidean position estimation RMSE: 0.154 m.
- Q: `diag([0.001 0.001 0.00001 0.003 0.003].^2)`; current sensor noise and estimator parameters are unchanged.
- The speed planner and controls receive estimates; tyre memories are predicted internally rather than read from the physical plant. No obstacles are introduced, so emergency intervention is inactive.

The original local video and simulation data remain in the project. A successful simulated lap is not experimental validation. To remove this temporary publication, remove `video-25` from both gallery catalogues, remove this folder, and adjust the featured count/build assertion. No automatic removal date has been set.
