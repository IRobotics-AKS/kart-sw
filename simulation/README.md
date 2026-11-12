# sim

Standalone Python package for the MuJoCo core. It currently provides a
**placeholder** Top Kart Dreamer model (`models/dreamer/dreamer.xml`) with three
cameras (front, left at +45 degrees, right at -45 degrees), a generated demo
track (`worlds/`, from `sim/track.py`), and helpers in `sim/scene.py` to load,
reset, drive, and render. All kart dimensions, camera mounts and fields of view,
tire friction, and actuator gains are guesses marked PLACEHOLDER until measured.
The demo track is a generic loop, not the AKS track. There is no ROS adapter,
sensor noise model, IMU/GNSS, or CLI yet.

Render the three camera views (Linux headless needs `MUJOCO_GL=egl`):

```sh
.venv/bin/python simulation/scripts/render_cameras.py render_out
```

Regenerate the track after editing `sim/track.py`: `python -m sim.track`.

From the repository root, with Python 3.12:

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -e simulation
.venv/bin/python -c "import sim; print(sim.__file__)"
```

MuJoCo 3.16.0 remains a candidate in the design documents. Verify its availability,
platform support, and behavior before adding an exact runtime dependency here.
ROS is not required to build or import this scaffold. The ROS adapter belongs in
`src/simulation/sim_ros`; firmware and physical communication do not belong here.

Add model loading, stepping, reset, and rendering before introducing a CLI. Keep
measurement outputs separate from evaluation truth. Invalidate commands and
supervisor sessions on reset, and measure pacing against a monotonic wall clock.

The wheel installs the current asset notes under `share/sim/` in the install
prefix. Update the explicit data-file lists when adding MJCF/assets and check
their presence in a built wheel; an editable install alone does not verify this.
