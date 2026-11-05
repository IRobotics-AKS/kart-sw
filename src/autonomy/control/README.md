# control

Consume estimated motion and a trajectory; publish requests to the vehicle manager.

`algorithms/pure_pursuit.py` implements geometric path tracking:
- a pose and an ordered path in
- a road-wheel steering angle
- a speed reference out

It holds no state, imports nothing from ROS, and is covered by
`tests/test_pure_pursuit.py`, which runs without a ROS environment.

It stops short of actuator commands on purpose. `docs/ros-structure.md` records
that the `ActuatorCommand` fields depend on whether the real boundary is current,
torque, throttle/brake or speed, so the node that would publish `control/request`
waits on that decision. Steering angle and speed reference are true under every
answer; converting them into actuator units is a downstream concern.

This is an installable ament_python package with no executable. 
