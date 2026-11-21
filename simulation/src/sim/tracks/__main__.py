"""Generate a random track and open it in the MuJoCo viewer, or save it.

From the repository root (on macOS use .venv/bin/mjpython for the viewer):

    .venv/bin/python -m sim.tracks --seed 7                     # view a track
    .venv/bin/python -m sim.tracks --edges cones --obstacles 4 --moving 1
    .venv/bin/python -m sim.tracks --seed 7 --out track.json    # save, no viewer
    .venv/bin/python -m sim.tracks --load track.json            # view a saved track
"""
import argparse
import sys
import time

import mujoco

from . import world
from .format import EDGE_STYLES, Track
from .generator import generate


def main(argv=None):
    ap = argparse.ArgumentParser(prog="python -m sim.tracks", description=__doc__.splitlines()[0])
    ap.add_argument("--seed", type=int, default=None, help="random if omitted")
    ap.add_argument("--layout", default="kart", choices=("kart", "flowing"),
                    help="kart: compact with hairpins (default); flowing: open and sweeping")
    ap.add_argument("--edges", default="mixed", choices=("mixed",) + EDGE_STYLES)
    ap.add_argument("--runoff", default="mixed", choices=("mixed", "grass", "concrete"))
    ap.add_argument("--obstacles", type=int, default=0, help="obstacles on the track (default 0)")
    ap.add_argument("--moving", type=int, default=0, help="how many of them slide across")
    ap.add_argument("--spread", action="store_true",
                    help="spread obstacles round the lap instead of one obstacle zone")
    ap.add_argument("--free-cones", action="store_true", help="edge cones can be knocked over")
    ap.add_argument("--load", help="open a saved track JSON instead of generating one")
    ap.add_argument("--out", help="save the track JSON here (and skip the viewer)")
    ap.add_argument("--kart", default=str(world.KART), help="kart MJCF (default: models/dreamer/kart.xml)")
    a = ap.parse_args(argv)

    track = Track.load(a.load) if a.load else generate(
        a.seed, edges=a.edges, runoff=a.runoff, obstacles=a.obstacles, moving=a.moving,
        layout=a.layout, zone=not a.spread)
    styles = sorted({x for s in track.sections for x in (s.left, s.right)})
    print(f"track seed {track.seed}: {track.length:.0f} m, width {track.width.min():.1f}-"
          f"{track.width.max():.1f} m, edges {', '.join(styles)}, {len(track.obstacles)} obstacles")
    if a.out:
        track.save(a.out)
        print(f"saved {a.out}")
        return
    model = world.build(track, a.kart, a.free_cones).compile()
    data = mujoco.MjData(model)
    view(model, data)


def view(model, data):
    import mujoco.viewer
    try:
        handle = mujoco.viewer.launch_passive(model, data)
    except RuntimeError as e:
        sys.exit(f"{e}\nOn macOS run this with mjpython instead of python.")
    with handle as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = model.body("chassis").id
        v.cam.distance, v.cam.elevation = 6.0, -18.0
        start = time.monotonic()
        while v.is_running():
            world.update(model, data)
            mujoco.mj_step(model, data)
            ahead = data.time - (time.monotonic() - start)
            if ahead > 0:
                time.sleep(ahead)
            v.sync()


if __name__ == "__main__":
    main()
