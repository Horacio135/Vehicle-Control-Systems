# Simulation video gallery

[Watch the gallery](https://horacio135.github.io/Vehicle-Control-Systems/#current).

The featured ESC V1 recording shows the controlled blue car and an orange semitransparent baseline car in 3D. The fixed-orientation camera follows the controlled car's X/Y position at 12 m height. Playback is normal speed, 25 fps. The baseline freezes when its recorded run aborts.

The test has 13 m lane-change transitions, perfect state and a constant 20 m/s reference, with nominal braking suppressed. ESC limits maximum sideslip to 1.88° and completes, but still contacts three cones. This is not an estimated-loop result or a cone-clearance pass. [Evidence and reproduction](../PROJECT/Results/ESC_V1_13m/README.md).

There are 12 players: three current recordings, one diagnostic and eight historical milestones, plus 11 archive links. The current ABS comparisons retain their own sensing/EKF configuration. The matching 2D ESC view is linked from the catalogue and results page.

Edit `catalog.json` and synchronize the embedded catalogue in `index.html`; run `python gallery/build_gallery.py`. The existing workflow validates and deploys media to GitHub Pages. Videos load only after Play is clicked.
