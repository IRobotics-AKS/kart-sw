# Decisions and open questions

Recorded: 2026-10-21; updated: 2026-10-27. The finalized project design now uses
a monorepo. Distinguish architecture decisions, unresolved hardware/version
choices, and implemented/verified software.

## Confirmed user requirements and plans

| Item | Status |
| --- | --- |
| Autonomous Top Kart Dreamer | Project target |
| MuJoCo simulation | Selected by the user |
| Repository layout | Monorepo; supersedes the initial multi-repo requirement |
| Controls separate from simulation | Separate ROS package/library boundaries within the monorepo |
| IMU and GNSS | BNO055 breakout and ArduSimple GNSS RTK Starter Kit with ZED-F9P named in its notes |
| Three cameras | One front OAK-D S2 and two side OAK-1 Lite cameras; Logitech BRIO explicitly excluded |
| Jetson on kart runs autonomy/controls | Jetson Orin Nano Super Developer Kit |
| Google Coral | Optional / undecided; user is unsure it is needed and has no identified workload |
| JetPack 7.x | Planned; exact release not chosen |
| TRAMPA VESC 75/300 MkIV | Planned motor controller; number and actuator roles unknown |
| Controls on workstation using simulation data | Required |
| Controls on kart's Jetson using simulation data | Required; simulated commands return to MuJoCo |

## Architecture baseline (2026-10-24)

- One repository for shared contracts, MuJoCo, autonomy, hardware integration,
  MCU firmware, and deployment. The current checkout is named `kart-sw`;
  scaffolding did not rename its directory or remote.
- Standalone MuJoCo Python project plus a ROS adapter; production autonomy never
  imports simulator internals or hardware communication protocols.
- Same autonomy source and ROS interfaces across all three required modes.
- Separate simulation, autonomy, hardware, and optional recorder images/services.
- Compose per host, ROS launch inside services, optional systemd for boot.
- Three bringup packages with separate dependency sets so autonomy does not pull
  simulation/hardware dependencies merely to install launch files.
- Physical actuator bridge absent in simulation; real actuators remain disarmed.
- Google Coral is optional and excluded from initial dependencies; BRIO is excluded.

The full target design is in [architecture.md](architecture.md),
[ros-structure.md](ros-structure.md), and [docker-deployment.md](docker-deployment.md).
This finalizes software organization, not unspecified physical actuator behavior.

## Superseded decisions

The 2026-10-21 plan used five repositories and a copyable bringup handoff. The
user subsequently requested a monorepo and a consolidated project setup. Preserve
the logical component boundaries as packages. `handoff/kart-bringup/` is historical
and must not override the current root documents or prompt creation of another repo.

## Recommendations still requiring hardware choices

A microcontroller supervisor between Jetson and VESCs remains recommended. Separate
upstream/downstream CAN buses are a target option; USB serial is a bench option.
MCU model, transport, command semantics, current/steering limits, watchdog durations,
and physical stopping sequence remain unresolved. No GPIO/CAN wiring or motor
configuration is finalized by this software architecture.

## Provisional version baseline

This is the recommendation recorded on 2026-10-21, not an installation lock or
validation result. Recheck compatibility before installing or publishing images.

| Component | Proposed baseline | Qualification |
| --- | --- | --- |
| MuJoCo | `3.16.0` | Pin exactly when implementation begins; test physics and rendering |
| ROS 2 | Jazzy | Keep simulation and Jetson on the same distribution |
| Isaac ROS | `4.6.0` | Optional selected perception packages; not needed by the simulation core |
| JetPack | `7.2` compatibility baseline | User specified only 7.x; hardware/carrier and patch release need confirmation |
| Ubuntu | `24.04` | Target environment for the proposed ROS/Isaac stack |
| Python | `3.12` | Already selected in this scaffold; validate with the chosen ROS environment |

JetPack 7.2.1 was available when discussed, but the cited Isaac ROS 4.6 matrix
explicitly named 7.2. Do not treat the patch release as validated here. Isaac ROS
5.0 / ROS 2 Lyrical was discussed as a newer alternative, not selected. Jazzy/4.6
was recommended partly because the camera stack was unresolved and Luxonis
explicitly documented Jazzy support.

References checked during the discussion:

- [MuJoCo changelog](https://mujoco.readthedocs.io/en/stable/changelog.html)
- [Isaac ROS 4.6 requirements](https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html)
- [Isaac ROS release notes](https://nvidia-isaac-ros.github.io/v/release-5.0/releases/index.html)
- [JetPack downloads](https://developer.nvidia.com/embedded/jetpack/downloads)
- [DepthAI ROS](https://docs.luxonis.com/software-v3/depthai/ros)
- [Planned VESC model](https://trampaboards.com/vesc-75v-300a-mkiv--300-each-p-36236.html)
- [VESC CAN protocol and timeout](https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md)
- [ROS clock design](https://design.ros2.org/articles/clock_and_time.html)

## Open questions before finalizing integration

1. Confirm the listed Orin Nano Super Developer Kit's installed configuration,
   storage, power/cooling, and any carrier-board changes.
2. Which exact JetPack image is supported by that hardware?
3. Confirm exact sensor variants/board revisions, interfaces, rates,
   synchronization, calibration, and mounting transforms for the identified
   hardware. Will camera-side inference be used? What is the GNSS correction
   source and delivery path? What Coral variant/workload is intended?
4. How many VESCs are used, and for propulsion, steering, or other actuators?
5. What motors, gear ratios, encoders, steering mechanism, battery/BMS, and
   physical current/voltage limits apply?
6. How will mechanical braking and an independent emergency stop be actuated?
7. Which MCU and Jetson-to-MCU transport will be used? What is the final command
   boundary and feedback contract?
8. What operating speeds, deadlines, sensor rates, and stopping requirements must
   be met? Determine timeout values from these requirements and measurements.
9. Which workstation OS/architectures/GPUs must support the full stack?
10. Which Isaac ROS packages are actually needed for the chosen perception stack?

## Implementation status

Layout update, 2026-10-27: the user requested removing the redundant standalone
project directory beneath `simulation/`. Its `pyproject.toml`, assets, tests, and
`src/sim/` now live directly under `simulation/`. The import remains `sim`;
the separate ROS bridge remains in `src/simulation/sim_ros/`. Build commands,
Docker COPY/install paths, CI, and scaffold checks follow the flattened layout.

Naming update, 2026-10-24: the user requested unprefixed component names.
All ROS packages, Python modules, their directories/resources, and corresponding
build/launch/deployment references now use names such as `control`, `interfaces`,
and `sim_ros`. The standalone Python distribution and module are both `sim`.
Compose configuration uses `NAMESPACE` and `REPO_ROOT`. This renames the scaffold
without changing its runtime behavior or component boundaries.

Hardware update, 2026-10-24: the user supplied a parts list and explicitly
excluded its Logitech BRIO entry. The current inventory is in
[hardware.md](hardware.md). This resolves the previously unknown Jetson and
sensor model names, but does not confirm installation or integration testing.

Scaffold update, 2026-10-24: added the standalone `simulation` Python
package, 13 ROS package skeletons with install metadata, guarded bringup launch
files, four Docker build recipes, four host-specific Compose templates, a
development source-mount override, and configuration/release worksheets. Firmware
has a separate reserved directory. No original root Python files existed to move.

The simulator wheel and eight ROS Python package wheels were built and inspected;
the standalone wheel was installed/imported in a clean Python 3.12 environment.
Static dependency checks and Compose rendering checks pass. ROS/ament/colcon are
not installed locally and the Docker daemon is unavailable, so ROS builds, launch
execution, container builds, and runtime integration are not verified here.

MuJoCo integration, shared message definitions, calibrated description, executable
nodes, MCU firmware, and control integration remain unimplemented. All launch
entry points fail explicitly before starting processes. Live mode exclusion,
readiness, arming, restart policy, and physical device mappings remain future work.
No software candidate has been promoted to a validated release pin. License and
maintainer metadata are explicit placeholders pending project-owner decisions.
The historical handoff remains unchanged and is not a current deployment.

## Maintaining this record

When a decision changes, record the date, new choice, reason, and affected
packages. Deployment pins belong under `deploy/`; shared ROS definitions belong
in `src/common/interfaces/`. Keep documents consistent with the actual
implementation and record verification evidence before calling versions tested.
