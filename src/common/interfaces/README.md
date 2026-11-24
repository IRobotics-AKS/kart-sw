# interfaces

Generates `SimActuatorCommand` for simulation-only direct motor efforts. It is
never used on `vehicle/command`; the physical actuator/status and trajectory
contracts remain unresolved. See [bridge contract](../../simulation/sim_ros/README.md)
for frames, units, QoS, validity and reset behavior. No executable node lives here.
