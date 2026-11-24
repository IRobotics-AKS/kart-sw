# Workstation simulation and deployment

The local Docker workflow runs **MuJoCo and its ROS 2 Jazzy bridge on a developer's
computer**. The real Jetson Orin Nano stays on the kart; this workflow does not
connect to it or configure its operating system, containers, sensors or actuators.
The current demo is a synthetic test body, not a calibrated autonomous kart.

## First run on a Mac

1. Install [Docker Desktop](https://docs.docker.com/desktop/setup/install/mac-install/)
   for your Mac's chip (Apple Silicon or Intel), then open it and wait for the
   engine to start. Docker Desktop includes Compose; use Compose v2.20 or newer.
2. Clone this repository and check out the team's simulation branch.
3. From the repository root:

```bash
./tools/sim-docker up
./tools/sim-docker smoke
```

`up` builds the image and starts the simulator in the background, waiting for a
ROS clock heartbeat. The first build downloads ROS and dependencies and takes
longer; subsequent builds reuse Docker's cache. No native ROS, Python virtualenv,
Ubuntu VM setup, NVIDIA runtime, or per-user environment file is required.

Docker selects Linux arm64 on Apple Silicon and Linux amd64 on Intel. The official
ROS Jazzy/Ubuntu 24.04 base is pinned to a verified multi-architecture image digest.
MuJoCo is pinned in `simulation/pyproject.toml`. Native Linux Docker can use the
same commands. Host operating system, CPU and network restrictions still need
validation on each team's machines; see the verification record below.

This is **headless simulation**: no graphics window opens. Inspect ROS data/logs
with the commands below. Desktop rendering/XQuartz, remote Jetson simulation,
perception GPU pipelines and full autonomy are outside this workstation setup.

## Everyday commands

```bash
./tools/sim-docker logs                  # add --follow to stream logs
./tools/sim-docker exec ros2 topic list --no-daemon
./tools/sim-docker exec ros2 topic echo /kart/sensors/imu/data --qos-reliability best_effort --once
./tools/sim-docker exec ros2 topic echo /kart/sim/ground_truth/odometry --qos-reliability best_effort --once
./tools/sim-docker demo                  # test-only 10 N force burst, two seconds
./tools/sim-docker reset                 # pause, reset t=0, leave paused
./tools/sim-docker resume                # old commands remain invalid
./tools/sim-docker smoke                 # requires advancing simulation
./tools/sim-docker shell                 # interactive shell with ROS sourced
./tools/sim-docker test                  # core + real ROS tests, isolated container
./tools/sim-docker down                  # stop this checkout's container
```

`demo` applies the included fixture's named motors; use `exec ros2 run sim_ros
send_efforts ...` for another model. Expired commands become zero effort, not
braking or a speed hold. See the [bridge contract](../src/simulation/sim_ros/README.md)
for topics, units, limits, time, QoS and session/reset behavior. `reset` does not
reset downstream estimators/controllers. The local workflow launches neither.

The wrapper resolves its own checkout path, so it works from another directory
when called by absolute/relative path. Paths containing spaces are supported.
A path-derived Compose project name separates simultaneous checkouts. Set
`KART_SIM_PROJECT=some-local-name` consistently for every command only if you want
an explicit name. Do not reuse a name for two running checkouts. Unrelated `.env`
files and the old Jetson/live template settings are not loaded.

Source is copied and built into the image. After changing Python, messages,
MJCF or configuration, rerun `up` to rebuild/recreate it. `exec` commands use the
installed build; there are no hidden host virtualenv or source bind mounts.
No host files are created by the container. Logs and runtime state are ephemeral;
`down` removes the container, and the next start has a new command session.

For a model committed under `simulation/` or a configuration installed by
`sim_bringup`, select **container paths**, not another developer's laptop paths:

```bash
KART_SIM_MODEL=/opt/kart/simulation/models/example.xml \
KART_SIM_CONFIG=/opt/kart/install/sim_bringup/share/sim_bringup/config/example.yaml \
  ./tools/sim-docker up
```

Those example files are placeholders; add the actual model/config first. Use the
same variables on subsequent Compose commands. These parameters only configure
the simulator. They cannot enable a physical bridge through this Compose file.

## Isolation and portability

`deploy/compose/workstation-local.yaml` contains just the simulation service.
It has no network interface beyond container loopback, published ports, hardware
access, privileged mode, Docker socket, host networking, source mount, or auto
restart. ROS discovery is localhost-only. Run ROS CLI tools using `exec`/`shell`
in that container, so Docker Desktop multicast and host DDS discovery are not
required. Multiple laptops do not discover each other's simulation or the kart.

The container runs as an unprivileged user with a read-only filesystem and a
writable temporary directory. The health check confirms ROS clock publication;
paused clock heartbeats are healthy. This is process/data availability, not
vehicle readiness. `smoke` additionally checks advancing clock, IMU, truth and
session data. No components are armed or physical commands published.

The separate `kart-live.yaml` and `jetson-sim.yaml` files remain unvalidated target
templates. They are never selected by `tools/sim-docker`. The existing
`workstation-sim.yaml`/`simulator-server.yaml` files describe future Linux/distributed
modes; use **workstation-local.yaml** through the wrapper for this Mac workflow.
MCU firmware continues to build separately. ARM64 image support does not establish
JetPack, Jetson GPU, camera-driver or physical deployment compatibility.

## Troubleshooting and verification

- Run `./tools/sim-docker doctor` to check Docker/Compose and daemon availability.
  If the engine is unreachable, open Docker Desktop and check `docker context ls`.
- If the build fails downloading from Docker Hub, PyPI, Ubuntu or ROS repositories,
  check the workstation's proxy/VPN/network settings. Builds need internet;
  simulation does not. Images are built locally and no registry login is required
  by these scripts (registry rate/account policies can still apply).
- If health/startup fails, use `status` and `logs`. A smoke timeout while paused
  is expected; use `resume`. Rebuild after changing package definitions.
- Do not set `DOCKER_DEFAULT_PLATFORM=linux/amd64` on Apple Silicon unless you
  intentionally want slower emulation; the scripts do not force a platform.
- The pinned base and MuJoCo version improve consistency, but apt/rosdep and
  transitive Python dependencies are not fully locked. This is a tested developer
  workflow, not a byte-for-byte reproducible release or a Jetson deployment.

Static checks (no daemon required):

```bash
python3 tools/check_workstation_deploy.py
python3 -m unittest discover -s tools/tests -v
python3 tools/check_deploy.py
```

The `Workstation simulation` CI workflow builds/tests on native amd64 and arm64
Linux runners and exercises launch, smoke, commands and reset. It does not publish
images or deploy to the kart. New CI runs must complete before claiming their
results. Local build/runtime results are recorded in `docs/docker-deployment.md`.
