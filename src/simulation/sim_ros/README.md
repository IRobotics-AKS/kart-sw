# sim_ros

Translate the standalone core API into measurements, status, selected-command input, reset, and the sole /clock publisher. Truth is evaluation-only and must not publish estimator-owned TF. Install sim separately; colcon does not install it. No ROS node or adapter API is implemented.

This is an installable ament_python package with no executable. See the root
`docs/ros-structure.md` for target topics, ownership, timing, and frame contracts.
Add runtime dependencies and console entry points with the implementation.
