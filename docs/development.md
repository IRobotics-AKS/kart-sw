# Developing the scaffold

Updated: 2026-10-24.

## Implemented structure

There are 13 discoverable ROS packages under `src/`: eight ament_python library
skeletons and five ament_cmake interface/description/bringup packages. Each has
package/install metadata. There are no executable ROS nodes, custom message
definitions, measured vehicle models, physical drivers, or control algorithms.
The simulator is a separate Python project; colcon does not install it.

Bringup launch files expose arguments, then raise a clear scaffold error before
starting processes. This permits package/argument discovery without implying a
working system. Do not remove the guard until that service's runtime and required
configuration are implemented. Hardware must continue to reject simulation time.

## Local inspection

From the repository root:

```sh
python3 tools/check_scaffold.py
python3 tools/check_deploy.py
bash -n deploy/docker/build-workspace.sh deploy/docker/entrypoint.sh
```

The first checks package ownership, transitive dependency boundaries, Python
syntax/import isolation, and colcon exclusions with the standard library. The
second uses Docker Compose to render all four modes and both supported development
overlays, checking clock selection, service isolation, device access, and image
source dependency sets. It never starts containers.

## Standalone simulator package

Use Python 3.12 and a separate environment:

```sh
python3.12 -m venv .venv
.venv/bin/python -m pip install -e simulation
.venv/bin/python -c "import sim; print(sim.__file__)"
.venv/bin/python -m pip wheel --no-deps --wheel-dir /tmp/kart-wheels simulation
```

This installs an empty simulator namespace and asset notes, not MuJoCo or a
working simulator. Verify the candidate MuJoCo version/platform before adding an
exact dependency. Update the wheel's explicit data-file list when adding MJCF,
worlds, or configuration; test installed assets outside the source checkout.

## ROS builds on the target Linux environment

Install/validate ROS 2 Jazzy, Python 3.12, rosdep, and colcon on the selected
Ubuntu 24.04 environment first. The following builds the autonomy package
skeletons after their declared dependencies have been installed:

```sh
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src
colcon build --base-paths src --packages-up-to autonomy_bringup
source install/setup.bash
ros2 launch autonomy_bringup autonomy.launch.py --show-args
```

Select `sim_bringup` or `hardware_bringup` for the corresponding
service. Scope dependency installation to that service's package closure too;
installing every repository dependency can pull unrelated simulation/driver
requirements into autonomy. Each Docker recipe copies only its service closure
before running rosdep. No vendor drivers are pinned or installed by this scaffold.

For simulation, explicitly install `simulation` into the Python
environment used to build/run the adapter. The simulation Docker recipe does this
before colcon. The firmware and handoff directories are excluded from colcon.
Use `--base-paths src` for deliberate ROS-only discovery.

The package manifests contain `TODO` license metadata and an
`example.invalid` maintainer address. Resolve these before publishing packages;
no license grant or real maintainer identity is inferred from this scaffold.

## Container templates

See [image instructions](../deploy/docker/README.md) and
[Compose instructions](../deploy/compose/README.md). Builds require an explicitly
chosen ROS base image; no unverified NVIDIA image tag or registry is supplied.
Compose inspection uses local example image tags and does not pull/build them.

Simulation modes contain no physical bridge or device mapping. The live template
also has no physical-device access yet. Launch guards intentionally prevent
runtime startup. Recording is an optional help-only profile until topics, clock,
storage path, and retention are selected. No systemd boot unit is installed.

Release pinning, a Jetson mode lock, runtime readiness/freshness checks, critical
node exit handling, disarmed recovery, hardware permissions, and bounded recording
remain implementation requirements. These templates do not establish them.

## Validation record

Local scaffold verification on 2026-10-24:

- Standard-library structure/dependency/import checks passed for all 13 packages.
- Docker Compose rendered all four modes and two development overlays; isolation,
  clock, device-access, and image dependency-closure checks passed.
- Shell scripts passed Bash syntax checks.
- Built the standalone simulator wheel and eight ROS Python package wheels;
  inspected package resources and installed the standalone wheel into a clean
  Python 3.12 environment, then imported it outside the repository.
- ROS/ament/colcon are unavailable locally and the Docker daemon is not running.
  ROS builds, actual launch execution, image builds, simulation, cross-host timing,
  and hardware behavior have not been tested. CI has been added but not run remotely.

## First implementation steps

1. Specify the shared trajectory, command, status, reset, and arming contracts,
   including clocks, units, frames, validity, QoS, and ownership.
2. Measure nominal geometry and mounts; implement the standalone MuJoCo model and
   sensor/actuator behavior without ROS or hardware imports.
3. Implement the simulation adapter and common autonomy/manager nodes, then
   replace launch guards as their services become operational.
4. Close a workstation loop using estimated state and explicit test trajectories;
   use simulation truth only for scoring. Test reset and stale-command behavior.
5. Build/validate pinned images and remote Jetson simulation before adding physical
   integration or boot startup. Resolve the hardware questions in
   [hardware.md](hardware.md) and [decisions.md](decisions.md).
