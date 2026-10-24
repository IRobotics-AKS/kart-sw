# Agent instructions for the kart project

Read `docs/architecture.md`, `docs/decisions.md`, `docs/hardware.md`,
`docs/ros-structure.md`, and `docs/docker-deployment.md` before architectural work.
These documents preserve project context across chats. Update them when the user
changes requirements. The monorepo design supersedes the earlier multi-repo plan.

## Scope and boundaries

- This is the source repository for the autonomous Top Kart Dreamer system.
  Its current checkout is named `kart-sw`; do not rename its remote or
  directory unless requested.
- Keep MuJoCo core code in `simulation/sim/`, independent of ROS. Keep its
  ROS adapter in `src/simulation/sim_ros/`.
- Use component names without a shared vehicle prefix for ROS packages and
  Python modules (for example, `control`, `interfaces`, and `sim_ros`).
- Keep deployed autonomy in `src/autonomy/`, shared contracts/description in
  `src/common/`, physical integration in `src/hardware/`, operating-mode logic in
  `src/system/`, and ROS launch/configuration in `src/bringup/`.
- MCU firmware lives in `firmware/` and builds separately. Container deployment
  lives in `deploy/`. Shared source does not imply a single image or process.
- The directory map is a target design. Do not claim unimplemented packages,
  launch files, Docker images, models, or interfaces already work.

## Runtime rules

- MuJoCo remains the simulation engine. Isaac ROS is optional accelerated ROS
  functionality; it does not imply Isaac Sim.
- Use the same autonomy source and ROS interfaces for workstation simulation,
  Jetson simulation testing, and real driving. Control code must not import
  MuJoCo, driver SDKs, or the MCU wire protocol.
- Keep simulator truth separate from sensor measurements and estimated state.
- Specify frames, units, QoS, timestamps, validity, reset behavior, and publisher
  ownership. Never silently add ideal feedback or an unimplemented speed loop.
- Simulation uses `/clock`. MCU watchdogs use their own monotonic clock. Measure
  wall-clock performance separately from simulation time.
- Simulation commands return to MuJoCo. Simulation deployment must not launch the
  physical actuator bridge, arm the kart, or enable a physical command path.
- Container startup/recovery is not vehicle readiness or permission to rearm.
- The vehicle manager coordinates ROS readiness and command selection; independent
  MCU supervision and stopping hardware remain separate responsibilities.

## Hardware and working practices

- Hardware details and unknowns are in `docs/hardware.md`. Exclude Logitech BRIO.
  Google Coral is optional/undecided and is not an initial runtime dependency.
- Do not invent actuator limits, MCU choice, final transport, camera calibration,
  GNSS corrections, or real braking behavior.
- Software versions are target candidates pending installation/integration checks.
- Preserve user work and implement only the requested scope. Run checks relevant
  to each change and report which checks could not be run.
- `handoff/kart-bringup/` and its zip are superseded historical material, not
  current instructions or a reason to create a separate bringup repository.
