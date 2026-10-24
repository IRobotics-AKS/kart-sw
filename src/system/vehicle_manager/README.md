# vehicle_manager

Coordinate fresh inputs, calibration/transforms, estimator validity, selected command source, and reported vehicle state. Startup/restart/reset must invalidate prior arm sessions. MCU supervision and independent stopping remain separate. No manager, arming service, or command forwarding is implemented.

This is an installable ament_python package with no executable. See the root
`docs/ros-structure.md` for target topics, ownership, timing, and frame contracts.
Add runtime dependencies and console entry points with the implementation.
