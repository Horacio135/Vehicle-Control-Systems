# Simulation video gallery

[Watch the gallery](https://horacio135.github.io/Vehicle-Control-Systems/#current).

The first official recording is the **33-corner complete loop with noisy sensors, EKF, ABS, TC and ESC**: 191.074 s lap, 0.505 m maximum actual path error and 0.154 m position estimation RMSE. Blue is the actual vehicle and the orange dotted outline is its estimated pose. This is the same video previously offered for temporary access, now promoted to official evidence. [Scope, data and reproduction](../PROJECT/Results/Extreme_33_Corners/README.md).

There are 14 players: five current recordings, one diagnostic and eight historical milestones, plus 14 archive links. The perfect-state ESC moose test and fourteen-corner ABS comparisons retain their own configurations. The four additional extreme-circuit videos are linked in the archive rather than expanding the compact featured grid; their captions identify perfect-state inputs explicitly.

Edit `catalog.json` and synchronize the embedded catalogue in `index.html`; run `python gallery/build_gallery.py`. The existing workflow validates and deploys media to GitHub Pages. Videos load only after Play is clicked. Publishing or rendering does not alter any dynamics/controller equations.
