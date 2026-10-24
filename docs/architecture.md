# Finalized project structure: kart monorepo

Updated: 2026-10-24. This is the target project architecture requested by the user.
It supersedes the earlier five-repository plan. Package/build skeletons, guarded
launch files, and deployment templates now exist. Simulator behavior, autonomy,
physical integration, and validated deployment images are not implemented yet.

## System scope

One repository holds the MuJoCo simulator, ROS autonomy, hardware integration,
MCU firmware, and deployment configuration for an autonomous Top Kart Dreamer.
The current checkout is named `kart-sw`; `kart/` below is a logical project name.
Scaffolding did not rename the directory or remote.

The selected computer is a Jetson Orin Nano Super Developer Kit. Sensors are one
front OAK-D S2, two side OAK-1 Lite cameras, a BNO055 breakout, and an ArduSimple
GNSS RTK Starter Kit with ZED-F9P named in its notes. Logitech BRIO is excluded.
Google Coral is optional/undecided and is absent from the initial software stack.
See [hardware.md](hardware.md) for remaining physical integration details.

## Directory ownership

```text
kart/
  AGENTS.md
  README.md
  docs/
    architecture.md
    decisions.md
    hardware.md
    ros-structure.md
    docker-deployment.md
  simulation/
    sim/
      pyproject.toml
      src/sim/               ROS-independent MuJoCo library
      models/dreamer/             MJCF and simulation assets
      worlds/                    Tracks and obstacles
      config/                    Noise, physics, scenarios
      tests/
  src/                           ROS workspace package discovery root
    common/
      interfaces/
      description/
    simulation/
      sim_ros/
    autonomy/
      perception/
      localization/
      planning/
      control/
    hardware/
      sensor_drivers/
      vehicle_bridge/
    system/
      vehicle_manager/
    bringup/
      autonomy_bringup/
      sim_bringup/
      hardware_bringup/
  firmware/
    vehicle_supervisor/          Separate MCU toolchain
  deploy/
    docker/                     Platform-specific image recipes
    compose/                    Four explicit host deployments
    config/                     Middleware and release configuration
    systemd/                    Optional Jetson boot integration
  tools/                        Developer/build helpers
  tests/integration/
```

Use three small bringup packages rather than one package that depends on every
subsystem. The autonomy image should not acquire a MuJoCo or physical-driver
runtime dependency just to install launch files. Each bringup package declares
and launches only its relevant dependency set; Compose assembles the services.

A package groups source/dependencies, a node is a runtime component, and a Docker
container is a deployment boundary. They need not have one-to-one relationships.
Do not make one container per ROS node.

## Build and dependency boundaries

- `simulation/sim` is a normal Python project. It loads, steps, resets, and
  renders MuJoCo without ROS. It owns simulated actuators, sensor behavior, and
  fault scenarios.
- `sim_ros` translates the simulation API to shared ROS contracts. Its image
  explicitly installs the Python core and its model assets before building ROS.
- `control` consumes estimated motion and a trajectory. It imports neither
  MuJoCo nor hardware communication code. Keep its algorithm testable separately
  from ROS callbacks.
- Shared ROS definitions live in `interfaces`; vehicle frames and nominal
  geometry live in `description`. Maintain one source of nominal dimensions
  and sensor mounting data; generate or validate simulation geometry against it.
- Physics-specific contact, tire/actuator response, solver settings, and noise
  belong to simulation. Do not assume generic contact friction validates kart
  tire behavior; model fidelity requires measurements.
- Use vendor camera/GNSS drivers as pinned external dependencies rather than
  copying their repositories into this one. Firmware builds independently.
- `colcon` builds selected ROS packages from `src/`; it does not automatically
  install the standalone simulation project or build firmware.

## Three operating modes

| Mode | Workstation | Jetson | Actuator destination |
| --- | --- | --- | --- |
| Workstation simulation | MuJoCo + autonomy | Not required | MuJoCo |
| Jetson simulation testing | MuJoCo | Same autonomy source | MuJoCo on workstation |
| Real driving | Optional monitoring | Autonomy + physical drivers/bridge | MCU and real actuators |

Simulation on the Jetson means executing the autonomy software on the physical
computer with simulated inputs. It does not enable real motors. Actual MCU
hardware-in-the-loop testing is a separate future configuration.

```text
                     Camera observations -> Perception ----+
Sensors or MuJoCo ---+                                     +-> Planning
                     IMU/GNSS/feedback -> Localization ----+       |
                                               |                  | trajectory
                                               +-----> Control <--+
                                                           |
                                                       request
                                                           v
                                                  Vehicle manager
                                                           |
                                                   selected command
                                                           v
                                      MuJoCo adapter OR physical MCU bridge
```

Physical drivers and simulation publish equivalent measurement/status interfaces.
The vehicle manager chooses the eligible command source and tracks readiness.
The MCU independently checks freshness/arming and enforces physical limits.
Neither the manager nor Docker replaces an independent stopping design.

## Implementation baseline

Continue with the previously proposed MuJoCo 3.16.0, ROS 2 Jazzy, Isaac ROS 4.6.0
where needed, Ubuntu 24.04/Python 3.12, and JetPack 7.2 compatibility baseline.
These are version candidates, not verified deployment pins; see
[decisions.md](decisions.md). Confirm the exact JetPack patch and driver/runtime
combinations on the Orin Nano before building release images.

Run MuJoCo on a workstation initially. The Jetson runs autonomy in live operation;
it does not need MuJoCo installed. Selected OAK inference may reduce Jetson load,
but benchmark actual pipelines before assigning workloads or adding accelerators.

## Delivery order and completion criteria

1. Define the minimum shared sensor, state, trajectory, and command contracts.
2. Implement simple MuJoCo dynamics and IMU/GNSS, plus the ROS adapter.
3. Close the workstation loop with localization, a fixed test trajectory, control,
   and the vehicle manager. Score against truth without using truth as feedback.
4. Package simulator and autonomy images; repeat tests from a clean environment.
5. Run the same autonomy revision on Jetson against remote MuJoCo. Check wall-clock
   deadlines separately from functional correctness in simulation time.
6. Integrate configured physical sensors and the MCU protocol once hardware
   decisions permit it, keeping live deployment separate and initially disarmed.
7. Expand camera/perception, planning, fault injection, and measured model fidelity.

For runtime contracts see [ROS structure](ros-structure.md). For images, host
configuration, launch ownership, and operational checks see
[Docker deployment](docker-deployment.md).

## Current checkout and historical material

The 2026-10-24 scaffold adds the standalone Python package and all 13 ROS package
directories with install metadata. There were no root `main.py` or `pyproject.toml`
files in this checkout to move. The simulator contains no dynamics or model;
interfaces contain no generated message/service definitions; autonomy and hardware
packages contain no executable nodes. Bringup exposes arguments but raises an
explicit error before starting processes. Geometry remains unmeasured.

Docker build recipes and four Compose templates are present but no images have
been built or validated. The scaffold does not implement operational mode locking,
readiness, command forwarding, MCU supervision, or arming. See
[development.md](development.md) for build instructions and verification limits.

The earlier `handoff/kart-bringup/` documents and both zip archives are retained as
historical snapshots; do not use them to assemble a separate bringup repository.
