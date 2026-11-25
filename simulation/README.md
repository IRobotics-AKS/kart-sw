# sim

Standalone ROS-independent MuJoCo runtime. It loads MJCF, steps and resets the
simulation, samples a co-located accelerometer/gyro with reproducible Gaussian
noise, exposes evaluation truth, and applies explicit bounded joint motor efforts.
The packaged `demo` is a synthetic box, not a calibrated kart model.

From the repository root, using the existing Python 3.12 environment with MuJoCo:

```bash
PYTHONPATH=simulation/src .venv/bin/python -m sim --model demo --duration 5
# macOS passive viewer requires MuJoCo's mjpython launcher:
PYTHONPATH=simulation/src .venv/bin/mjpython -m sim --model demo --viewer --duration 30
```

For a new environment install the package with `python -m pip install -e simulation`.
On Linux, `python -m sim --model demo --viewer --duration 30` runs the desktop
viewer. Headless runs do not need a display. The standalone CLI explicitly uses
zero sensor noise and zero motor effort; the ROS demo config adds synthetic noise.

`Simulation` requires explicit noise standard deviations and seed. Models must
contain a configured body and same-site accelerometer/gyro. Motors, if present,
must be named unit-gear hinge/slide motors with finite control limits containing
zero. `set_efforts` takes a complete named vector (hinge Nm, slide N); it does not
implement timing, arming, speed control or braking. The ROS bridge owns freshness,
session and sequence checking before calling it. Reset restores initial model
state, zeros controls and restarts the random seed.

MJCF worlds/geometry remain caller supplied. The real Dreamer model, sensor
calibration, GNSS/camera simulation and physical actuator semantics are pending.
See the [ROS bridge runbook](../src/simulation/sim_ros/README.md) for launch commands
and the full measurement/command contract.
