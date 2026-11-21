# Reviewing this study

Thanks for checking it. These are the places where a mistake would change the answer, roughly in order of importance.

## 1. Frames and signs
- Kart frame: rear axle centre on the ground, x forward, y left, z up. Check this against `simulation/models/dreamer/kart.xml` (the `chassis` body sits at z 0.1427 with the axle at its origin).
- Camera angles: yaw + turns the view to the left, pitch + tilts it down. See `sim/camplace/geometry.py::cam_rot`; the tests in `simulation/tests/test_camplace.py` pin this down.
- Ground points are moved into the kart frame in `sim/camplace/lap.py::edge_points`. A sign error there would mirror the track.

## 2. Kart blocking
- `sim/camplace/kart.py` casts rays from the camera position against every kart geom except the floor (`bodyexclude=0`), at the lap's steering angles plus full lock.
- `scripts/render_check.py` overlays the result on a MuJoCo render. `tests/test_blocking_map_matches_render` asserts at least 95% pixel agreement.
- Clearance uses 4000 random rays, and a point is treated as inside a part when more than half of them hit. Check the threshold makes sense.

## 3. Scoring choices (judgement calls; challenge them)
- Edge quality = min(1, 10 cm ÷ cm-per-pixel across the edge) at 1280 px (`config.GSD_THR_CM`, `config.WIDTH_PX`).
- Band weights: beside 0.30, near 0.30, mid 0.25, far 0.15 (`config.WEIGHTS`).
- Robust = the worse of the lap mean and the worst-10% pose, over pitch ±2° and roll ±1°.
- Limits: 0.80 m height cap, 15% kart in the image, 5 cm clearance, nothing behind the rear axle.
- "Good" = within 2% of the best set score. Each box is found with the other two cameras at their best. With all three at the worst corners at once, the set is still at 95% of best.

## 4. Lap model
- Speeds come from curvature (`config.SPEED`); steering from the bicycle model. The speed only affects which steering bin is used for blocking.
- Generated tracks have no racing line, so they're driven on 3 lines (`sim/camplace/track.py::generated`).

## Numbers to reproduce

`scripts/` is `simulation/studies/camera_placement/scripts/`; run from the repository root with the `sim` package installed.

| Command | Expected |
| --- | --- |
| `pytest -q simulation/tests/test_camplace.py` | 11 passed |
| `python scripts/check_set.py --set crossbar65` | score 0.9491, gap 0–8 m: 0.00 m p95 / 0.50 m max, kart in image 13% / 2% / 2% |
| `python scripts/check_set.py --set crossbar` | score 0.9335 (98.1% of 0.9511) |
| `python scripts/check_set.py --set best_purdue` | score 0.9511 |
| `python scripts/optimize.py` | best 0.9511; boxes as in `results/summary.json` |
| `python scripts/validate.py` | `results/validation.md` |

`results/summary.json`, `sets.json` and `joint.json` come from the original run (aks-gps `scripts/cam/refine.py`, 2026-11-19). This code was checked against them: same edge-point count (92,020) and the same set scores, on MuJoCo 3.16.0 (kart-sw's pin) and 3.17.0.

## Known gaps
See the Limits section in README.md. The biggest open questions are what the cameras will mainly be used for (the score is about track edges), and the real mount stiffness, blur and rolling shutter, which need footage from the real cameras.
