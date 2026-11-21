# Camera placement study

Where to mount the AKS kart's three cameras: one Luxonis OAK-D S2 at the front and two OAK-1 Lites at the sides, on the Top Kart Dreamer. The code simulates the CAD kart driving a lap and scores every candidate camera position and angle by how much of the track edge the cameras see. It counts what the kart's own body blocks and keeps the result robust to the kart pitching and rolling.

## Result

Put all three cameras on one crossbar just ahead of the rear axle, 0.75 m off the ground.

| Camera | x (m) | y (m) | z (m) | Turned outward | Tilted down |
| --- | --- | --- | --- | --- | --- |
| Front, OAK-D S2 | 0.05 | 0 | 0.75 | 0° | 6° |
| Left, OAK-1 Lite | 0.05 | +0.30 | 0.75 | 60° | 25° |
| Right, OAK-1 Lite | 0.05 | −0.30 | 0.75 | 60° | 25° |

x is forward, y left and z up, measured from the rear axle centre at ground level.

On Purdue, this set scores 98.1% of the best set found. The side cameras at 65° score 99.8%, but 60° keeps a 9° overlap with the front camera as margin for mounting error. Each camera can move inside a box and still score within 2% of the best:

| Camera | x (m) | y (m) | z (m) | Angles |
| --- | --- | --- | --- | --- |
| Front | 0 to 0.15 | −0.10 to 0.10 | 0.625 to 0.80 | 2.5–7.5° down, more when higher |
| Left | 0 to 0.30 | 0.30 to 0.50 | 0.45 to 0.75 | 60–65° out, 15–25° down |
| Right | 0 to 0.35 | −0.30 to −0.45 | 0.45 to 0.75 | 60–65° out, 20–25° down |

Pages with the full study and a 3D view of the kart (private to the owner; ask for access):
- Report: https://claude.ai/artifact/Xg5m15TW5MShVAbZ6FZKNe
- Layout for mechanical: https://claude.ai/artifact/R5ccmPGfaL5MyHd15LuZDE

## Multi-track check

The layout was found on the Purdue track and then checked on 5 tracks from the track generator (`sim.tracks`). See [results/validation.md](results/validation.md).

| Track | Width (m) | Tightest radius (m) | Crossbar % of best | Worst gap 0–8 m (m) |
| --- | --- | --- | --- | --- |
| purdue | 5.1–6.7 | 5.9 | 98.1 | 0.75 |
| kart_s101 | 8.0–8.7 | 7.6 | 98.1 | 0.00 |
| kart_s102 | 5.8–6.6 | 5.7 | 97.3 | 8.00 (see below) |
| kart_s103 | 6.2–6.7 | 6.1 | 97.4 | 0.50 |
| kart_s104 | 6.3–6.9 | 6.1 | 97.5 | 0.00 |
| flowing_s201 | 6.4–6.5 | 6.2 | 97.8 | 0.00 |

- The crossbar set scores 97.3–98.1% of the best found on every track. The 65° sides score 99.3–100%.
- On the generated tracks, the re-optimised best sits just outside the Purdue boxes: cameras at the 0.80 m cap and the front camera further forward (x 0.4–0.6 m). That gains at most 2.7%, so the layout doesn't need to change. It does suggest mounting near the top of the boxes is the safer choice.
- **kart_s102's 8 m gap is open.** It comes from 2 of 930 poses, where most of the edge "0–8 m ahead" lies beside or behind the kart. Every set tested has the same gap, including the best one found on Purdue. It looks like an artefact of driving a ±40% offset line through a 5.7 m corner rather than a real blind spot, but that hasn't been confirmed. With those 2 poses excluded, no gap is over 1 m.

## How it works

1. **Kart:** the CAD Dreamer model ([simulation/models/dreamer](../../models/dreamer)) loaded in MuJoCo.
2. **Lap:** kart poses every 1 m along a driven line. Speeds come from the line's curvature; steering from the kinematic bicycle (wheelbase 1.04 m).
   - Purdue is driven on its min-time racing line.
   - Generated tracks are driven on the centreline and on lines at ±40% of the half width.
3. **Edge points:** both edges, every 0.25 m, from 1.5 m behind the kart centre to 25 m ahead, in four bands: beside, near, mid, far.
4. **Quality per point and camera:**
   - It must be inside the camera's field of view (OAK-D S2 69°×55°, OAK-1 Lite 69°×54°).
   - It must not be blocked by the kart at that pose's steering angle. Blocking is ray-cast every 0.5°.
   - Quality = min(1, 10 cm ÷ ground size of one pixel across the edge), at an assumed 1280 px image width.
5. **Set score:** each point takes the best quality any camera gives it. Coverage per pose and band is the mean over both edges. Per band, the lap mean and the worst 10% of poses are averaged; each is the worst over the kart pitched ±2° and rolled ±1°. The bands are then weighted 0.30 / 0.30 / 0.25 / 0.15.
6. **Hard limits:** nothing behind the rear axle, nothing above 0.80 m, the camera at least 5 cm from any kart part, and the kart filling at most 15% of the image at any steering angle up to full lock.
7. **Search:** each camera is swept with the other two fixed, in turn, until none moves. Then each gets a fine grid, and its box is the largest block of fine-grid positions that are all within 2% of the best.

All the limits and assumptions are in [sim/camplace/config.py](../../src/sim/camplace/config.py).

## Run it

From the repository root, with the `sim` package installed (see [simulation/README.md](../../README.md)). `render_check.py` also needs Pillow.

```sh
S=simulation/studies/camera_placement/scripts
.venv/bin/python -m pytest -q simulation/tests/test_camplace.py   # about 5 s
.venv/bin/python $S/check_set.py                  # score the recommended set on Purdue (about 20 s)
.venv/bin/python $S/check_set.py --worst-corners
.venv/bin/python $S/render_check.py               # render the camera views with the blocking map in red
.venv/bin/python $S/optimize.py                   # the full search on Purdue, about 30 min with 6 workers
.venv/bin/python $S/validate.py                   # the multi-track check, about 45 min
.venv/bin/python $S/export_mesh.py runs/kart.json # kart mesh for the 3D pages
```

Library code is in `simulation/src/sim/camplace/`; scripts, data and results are in this folder. Outputs from `optimize.py` go to `runs/` here.

## Limits

- **Geometry only:** no lighting, glare, texture, lens distortion, motion blur or rolling shutter, and no detector.
- **Placeholder parts:** the nose, side pods, motor, battery and electronics are simple shapes in guessed positions. The camera bodies and any mount frame aren't modelled as blocking the view.
- **Assumed numbers:** the 0.80 m height cap, the 10 cm/px threshold and the 1280 px width were chosen by us, not taken from rules or the vision pipeline.
- **Track edges only:** the score is about seeing the edges. Obstacles on the track (cones, boxes, barrels) sit in the same area, so the layout covers them too, but there's no separate object score.
- **No other karts:** traffic isn't modelled.

## Data

- Kart: the shared CAD model in `simulation/models/dreamer`.
- `data/tracks/purdue_rtk`: built from the 2021 RTK edge survey of the Purdue kart track, and its min-time racing line.
- `data/tracks/generated`: the 5 validation tracks, made with `sim.tracks` by `scripts/validate.py`.
