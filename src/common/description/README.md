# description

Installable asset package; no calibrated geometry, URDF, Xacro, or RViz setup is
implemented. `config/geometry.example.yaml` is a measurement worksheet, not a
robot_state_publisher input. Null means unknown; never interpret it as zero.

This package is the future source of nominal dimensions and sensor mounting
transforms. Simulation must generate or validate geometry against that source.
Autonomy bringup will own one robot_state_publisher after the model exists.
