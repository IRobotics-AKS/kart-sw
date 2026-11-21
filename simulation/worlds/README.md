# worlds

Tracks are generated in code by `sim.tracks` (in `simulation/src/sim/tracks/`), so no
world files are stored here yet. A track can be saved as JSON and loaded again.

## Run

From the repository root (on macOS use `.venv/bin/mjpython` for the viewer):

```sh
.venv/bin/python -m sim.tracks                          # random track, mixed edges
.venv/bin/python -m sim.tracks --seed 7                 # the same track every time
.venv/bin/python -m sim.tracks --edges tyres            # one edge style everywhere
.venv/bin/python -m sim.tracks --obstacles 4 --moving 1 # obstacle zone, one sliding
.venv/bin/python -m sim.tracks --layout flowing         # open, sweeping layout
.venv/bin/python -m sim.tracks --seed 7 --out t.json    # save instead of viewing
.venv/bin/python -m sim.tracks --load t.json            # view a saved track
```

The viewer follows the kart. If `kart.xml` has actuators (the CAD kart has steer / drive /
brake), drive it with the Control panel sliders. Press 3 to show collision shapes.

In code:

```python
from sim.tracks import generate, world
track = generate(seed=7, edges="mixed", obstacles=3, moving=1)
model, data = world.compile(track)        # kart.xml + the track, kart on the start
# each step: world.update(model, data); mujoco.mj_step(model, data)
```

## Tracks

- **Layout** (`--layout`):
  - `kart` (default): like a kart circuit or a parking-lot course (the AKS tracks at AMP
    and Apex). A random region of grid cells grows in narrow arms and its outline becomes
    the centre line, so the lap folds into hairpins with parallel straights one gap
    apart. Corners are circular arcs; the grid is jittered and leaned so it is not boxy.
  - `flowing`: random points, convex hull, pushed-in midpoints, a smooth curve. More open.

  Both: 300-1000 m lap, 5-9 m wide, no corner tighter
  than 5 m radius (and always room inside a corner for barriers), 8-40 corners, a
  straight of at least 20 m with the start line in its middle, at least 5 m of grass
  between separate parts of the track. Layouts that fail are regenerated.
- **Edges** (`--edges`, per section and side in `mixed`):

  | style | looks like | kart hits it |
  |---|---|---|
  | `lines` | white painted edge line | no |
  | `kerbs` | edge line + red/white kerbs at corners (inside at the apex, outside on the exit) | no |
  | `cones` | 0.45 m orange cones with a white band, every 4 m (2.5 m in corners) | yes (fixed; `--free-cones` lets them fall over, slower) |
  | `tyres` | stacks of 3 tyres, 0.6 m tall | yes |
  | `concrete` | 0.8 m jersey barriers, 2 m blocks | yes |
  | `plastic` | 0.8 m red/white barrier blocks | yes |
  | `none` | bare asphalt edge | - |

- **Runoff** (`--runoff`): grass, or a 2.5 m concrete strip outside the edge.
- **Obstacles** (`--obstacles N`, off by default): cones, cardboard boxes or barrels on the
  driving surface. They go in one obstacle zone, as in the AKS time trial: a slalom on the
  straightest part of the lap (at least 30 m from the start), 6-10 m apart on alternating
  sides, always leaving a gap the kart fits through. `--spread` instead scatters them round
  the lap (25 m from the start, 15 m apart). `--moving M` makes M of them slide across the
  track (mocap bodies; `world.update` moves them).
- **Visuals:** asphalt, grass and concrete textures generated in code, chequered start
  line, sky, a sun with shadows that follows the kart, 1920x1080 offscreen buffer. No
  cameras are added yet.

## Notes

- `kart.xml` is not modified. The world loads it and adds the track; it only needs the
  names `floor`, `chassis` and `chassis_free`. Its floor stays the collision ground
  (hidden); the visible grass is a separate plane 2 cm lower so layers never flicker.
- Everything the kart can hit uses contype 1 / conaffinity 1, like the floor.
- Speed (on a laptop, fixed cones): about 45-70x real time. Free edge cones make each
  cone a body and can drop it to about real time.
- Saved tracks (JSON) hold the layout, sections and obstacles. The world itself is
  rebuilt from them; MJCF export is not supported because textures are generated in code.
