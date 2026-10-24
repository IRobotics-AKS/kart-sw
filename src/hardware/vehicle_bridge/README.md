# vehicle_bridge

Translate only the agreed shared command/status contract to the selected MCU protocol. Reject simulation time, expire commands, invalidate sessions on reconnect, and never rearm on restart. MCU, transport, actuator units/limits, and stop behavior are unresolved. No transport or actuator path is implemented.

This is an installable ament_python package with no executable. See the root
`docs/ros-structure.md` for target topics, ownership, timing, and frame contracts.
Add runtime dependencies and console entry points with the implementation.
