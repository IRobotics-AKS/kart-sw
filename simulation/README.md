# sim

Standalone Python package for the future MuJoCo core. It currently installs an
importable package and asset-directory notes; it has no simulator entry point,
model, sensor generator, command implementation, or runtime dependencies.

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
