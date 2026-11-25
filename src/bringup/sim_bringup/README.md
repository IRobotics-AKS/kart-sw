# sim_bringup

Launch the MuJoCo bridge with an explicit MJCF model and ROS parameter file.
See [bridge instructions](../../simulation/sim_ros/README.md) for installation,
commands, topics and the simulation-only actuator contract.

From the repository root after building and sourcing the workspace:

```bash
ros2 launch sim_bringup simulation.launch.py model:=demo \
  config_file:="$PWD/src/bringup/sim_bringup/config/bridge_demo.yaml"
```

Add `rendering:=true` for the MuJoCo desktop viewer. The demo is a synthetic
box with force/torque motors, not calibrated kart geometry. No model is selected
by default, and simulation requires `use_sim_time=true`. The legacy `scenario`
and `sensor_config` arguments remain reserved and reject nonempty values.
Autonomy/hardware launch remain guarded. No physical bridge is launched.
