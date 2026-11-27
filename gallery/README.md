# Simulation video gallery

[Watch directly in the browser](https://horacio135.github.io/Vehicle-Control-Systems/#current).

The gallery now has 12 players: two current modulated-ABS videos, one diagnostic and nine historical milestones. Ten further recordings remain archive links. Video files load only after Play is clicked.

The featured ABS off/on comparison keeps **TC on in both**, with sensors and EKF active. Both laps complete: 49.250 s without ABS, 47.294 s with modulated ABS. The second current video superposes actual and estimated positions at normal speed. Working Q returns to the reduced configuration; sensor noise is unchanged. No ESC yaw law is present.

The retained moose test is historical functional-boundary evidence: TC limits motor demand but does not guarantee yaw stability; ABS is enabled without braking demand. Earlier full-loop comparisons use the previous ABS and must not be confused with the current comparison.

## Maintenance

Edit `gallery/catalog.json`, synchronize the embedded catalogue in `gallery/index.html`, and run `python gallery/build_gallery.py`. The build verifies the two current entries and all selected media/posters. The existing GitHub Actions workflow deploys the page after pushes to main. The full catalogue records inclusion/exclusion reasons.
