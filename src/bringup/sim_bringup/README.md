# sim_bringup

Workstation MuJoCo adapter launch. This ament_cmake package installs its launch/configuration files.
It depends only on this service's ROS packages. The launch file exposes arguments
through `ros2 launch sim_bringup simulation.launch.py --show-args`.
Execution currently raises an explicit scaffold error before starting any process.

Implement the runtime nodes and validated configuration before replacing that
guard. Keep the simulation-time requirement; never add a physical bridge or device mapping.
