# hardware_bringup

Physical sensors and MCU bridge launch. This ament_cmake package installs its launch/configuration files.
It depends only on this service's ROS packages. The launch file exposes arguments
through `ros2 launch hardware_bringup hardware.launch.py --show-args`.
Execution currently raises an explicit scaffold error before starting any process.

Implement the runtime nodes and validated configuration before replacing that
guard. Keep the simulation-time rejection. Physical startup and recovery must remain disarmed.
