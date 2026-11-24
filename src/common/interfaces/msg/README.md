# Message definitions

`SimActuatorCommand.msg` is implemented for the explicitly simulation-only
`sim/actuator_command` topic. It carries a complete set of named joint motor
forces/torques, timestamp, session, sequence and validity. See
[bridge contract](../../../simulation/sim_ros/README.md).

Production `ActuatorCommand`, `VehicleStatus` and `Trajectory` remain proposed;
no final physical command semantics are inferred from the simulation test input.
