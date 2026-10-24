# localization

Consume measurements and publish estimated odometry. The local estimator owns odom to base_link; global localization owns map to odom. Never use simulation ground truth as implicit feedback. Estimator, covariance, origin, and reset behavior remain to be implemented.

This is an installable ament_python package with no executable. See the root
`docs/ros-structure.md` for target topics, ownership, timing, and frame contracts.
Add runtime dependencies and console entry points with the implementation.
