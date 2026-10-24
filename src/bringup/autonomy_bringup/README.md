# autonomy_bringup

Common autonomy stack and vehicle management. This ament_cmake package installs its launch/configuration files.
It depends only on this service's ROS packages. The launch file exposes arguments
through `ros2 launch autonomy_bringup autonomy.launch.py --show-args`.
Execution currently raises an explicit scaffold error before starting any process.

Implement the runtime nodes and validated configuration before replacing that
guard. Use the same autonomy code in both clock modes. Launch robot_state_publisher exactly once after geometry is available.
