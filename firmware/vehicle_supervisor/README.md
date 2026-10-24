# Vehicle supervisor firmware

Separate MCU project, outside colcon and Docker runtime. MCU model, toolchain,
transport, wire protocol, actuator roles/units/limits, watchdog duration, brake
sequence, and independent stop design are unresolved. No firmware is implemented.

Select those before adding board files or a build system. Use the MCU's monotonic
clock for freshness supervision; repeated stale commands must not refresh its
watchdog. Boot/reconnect/reset must invalidate prior sessions and remain disarmed.
Define feedback validity and clock conversion without treating uptime as ROS time.
