# interfaces

Installable ament_cmake/rosidl package skeleton. It currently generates **no
messages or services**. Standard measurement and odometry messages retain the
roles documented in `docs/ros-structure.md`.

Before adding Trajectory, ActuatorCommand, VehicleStatus, or services, agree on:
reference frames and units; source/measurement timestamps; session and sequence
identity; validity and expiration; QoS; reset/time-jump behavior; publisher and
consumer ownership; actual feedback availability; and the MCU command boundary.

Current, torque, throttle/brake, and speed are different contracts. Do not choose
one by writing placeholder fields. Do not expose an arm operation until the
manager/MCU handshake and independent stop responsibilities are specified.

List reviewed interface files explicitly in CMakeLists.txt; add their ROS
dependencies to both package.xml and rosidl_generate_interfaces.
