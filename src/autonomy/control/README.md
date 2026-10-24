# control

Consume estimated motion and a trajectory; publish requests to the vehicle manager. Keep algorithms under algorithms/ free of ROS callbacks, MuJoCo, SDKs, and the MCU wire protocol. No speed loop or actuator command semantics have been chosen.

This is an installable ament_python package with no executable. See the root
`docs/ros-structure.md` for target topics, ownership, timing, and frame contracts.
Add runtime dependencies and console entry points with the implementation.
