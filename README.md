# kart-sw

Monorepo for the autonomous Top Kart Dreamer: a ROS-independent MuJoCo core,
ROS 2 packages, separate MCU firmware, and deployment templates.

A MuJoCo runtime and ROS bridge now run a synthetic test body, publish clock/IMU/
truth, and accept simulation-only joint efforts. See the [bridge runbook](src/simulation/sim_ros/README.md)
for build and launch commands. Calibrated kart dynamics, production command
contracts, autonomy integration and firmware remain pending. Autonomy/hardware
launch are still guarded. This is not a simulated or real autonomous kart yet.

## Run the workstation simulator

Install/open Docker Desktop, then run from the repository root:

```bash
./tools/sim-docker up
./tools/sim-docker smoke
./tools/sim-docker down
```

This runs the headless test simulator and ROS bridge locally on Apple Silicon or
Intel Macs. It does not connect to the real Jetson on the kart. See
[workstation setup](deploy/README.md) for commands, testing and limitations.

## Layout

| Directory | Responsibility |
| --- | --- |
| `simulation/` | Standalone MuJoCo runtime and synthetic bridge test model |
| `src/common/` | Shared ROS interface and vehicle-description skeletons |
| `src/simulation/` | ROS adapter for the standalone simulator |
| `src/autonomy/` | Perception, localization, planning, and control |
| `src/hardware/` | Physical sensor integration and MCU bridge |
| `src/system/` | Vehicle readiness and command selection |
| `src/bringup/` | Separate autonomy, simulation, and hardware launch packages |
| `firmware/vehicle_supervisor/` | Future MCU project with its own toolchain |
| `deploy/` | Docker recipes, per-host Compose templates, release worksheets |
| `tools/`, `tests/integration/` | Scaffold checks and future integration tests |

## Check the scaffold

```sh
python3 tools/check_scaffold.py
python3 tools/check_deploy.py  # requires Docker Compose; no daemon needed
```

Python packages target Python 3.12. The checks themselves work with Python 3.9+.
See [development.md](docs/development.md) for standalone installation, scoped
colcon builds, image inspection, validation results, and implementation steps.
The [CI workflow](.github/workflows/scaffold.yml) runs structure, Compose, and
standalone packaging checks.

## Project context

- [Architecture and ownership](docs/architecture.md)
- [ROS interfaces, frames, timing, and launch contracts](docs/ros-structure.md)
- [Deployment design](docs/docker-deployment.md)
- [Hardware inventory and unresolved integration details](docs/hardware.md)
- [Decisions and version candidates](docs/decisions.md)
- [Agent instructions](AGENTS.md)

The historical [handoff](handoff/kart-bringup/README.md) and zip archives are retained
for context. The root documents supersede their earlier multi-repo plan. Software
versions remain candidates; maintainer email and license metadata are explicit
placeholders until the project owner selects them.
