# sim_ros

ROS 2 bridge for the standalone `sim` MuJoCo package. It loads and steps MJCF,
publishes simulated IMU readings, evaluation truth and `/clock`, and accepts
**simulation-only joint efforts**. The included demo is a falling/sliding box
with a yaw joint, not a calibrated Top Kart model or an autonomy demonstration.

## Docker on Mac workstations

From the repository root, use `./tools/sim-docker up` and then
`./tools/sim-docker smoke`. See the [workstation runbook](../../../deploy/README.md).
This runs the simulator and ROS together without a host ROS installation.

## Run on Ubuntu with ROS 2 Jazzy

From the **repository root** (`kart-sw`, not `kart-sw/src`), after installing
ROS 2 Jazzy, colcon, rosdep and Python 3.12 venv support:

```bash
source /opt/ros/jazzy/setup.bash
# Use a separate Linux ROS environment, not a virtualenv copied from macOS.
python3 -m venv --system-site-packages .venv-ros
source .venv-ros/bin/activate
python -m pip install -e simulation
rosdep install --from-paths src/common/interfaces src/simulation/sim_ros src/bringup/sim_bringup --ignore-src --rosdistro jazzy -y
python "$(command -v colcon)" build --base-paths src --packages-up-to sim_bringup
source install/setup.bash
ros2 launch sim_bringup simulation.launch.py \
  model:=demo \
  config_file:="$PWD/src/bringup/sim_bringup/config/bridge_demo.yaml"
```

For a desktop viewer add `rendering:=true` (requires a working desktop/OpenGL).
Headless is the default. Select `model:=/absolute/path/to/model.xml` and a reviewed
parameter file for a different model. No model/config is silently selected.
Autonomy and hardware bringup remain guarded; do not start them for this demo.
The local workstation Compose workflow supplies the packaged demo model/config;
the older Linux/distributed templates still require configuration.

In another terminal, source the same ROS, virtualenv and workspace environments:

```bash
ros2 topic echo /clock --once
ros2 topic echo /kart/sensors/imu/data --qos-reliability best_effort --once
ros2 topic echo /kart/sim/ground_truth/odometry --qos-reliability best_effort --once
ros2 run sim_ros send_efforts --effort x_force=10 --effort yaw_torque=0 \
  --duration 2 --ros-args -r __ns:=/kart
```

`send_efforts` reads the current clock and session and sends a finite burst.
Watch bridge warnings for rejection; it does not receive an acceptance ack.
Use one command sender on one host per session. It stops on a session change;
a pause/reset never automatically resumes the previous command intent. Defaults
are for the demo's 1x pacing; adjust rate/validity to the bridge's configured
limits for other pacing factors. All motors must be supplied, including zeros.

## Topics and ownership

Names below are relative to the configured namespace, except global `/clock`.
Run exactly one clock publisher and one sensor source in the ROS domain.

| Topic | Type | QoS | Contract |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | reliable, volatile, depth 1 | MuJoCo simulation time; bridge owns it |
| `sensors/imu/data` | `sensor_msgs/msg/Imu` | sensor-data: best effort, volatile, depth 5 | measurement in configured IMU site axes |
| `sim/ground_truth/odometry` | `nav_msgs/msg/Odometry` | sensor-data | evaluation only, never localization feedback |
| `sim/actuator_command` | `interfaces/msg/SimActuatorCommand` | reliable, volatile, depth 1 | simulation-only complete motor effort vector |
| `sim/session` | `std_msgs/msg/String` | reliable, transient local, depth 1 | current session ID; not an arm state |

Truth position/orientation use `world_frame` (default `sim_world`); twist is at
the body origin in `body_frame` axes. MuJoCo wxyz quaternions are converted to ROS
xyzw. IMU acceleration is specific force in m/s² (gravity included in the
stationary accelerometer reading); gyro is rad/s. The two MJCF sensors must be
on the same site on the chosen body. Configure site orientation and frame IDs
consistently with the project X-forward/Y-left/Z-up convention. No TF is emitted.

IMU orientation is unavailable (`orientation_covariance[0] = -1`), rather than
filled from truth. Noise is seeded independent Gaussian noise per sample/axis,
with covariance equal to its variance. Explicit zero noise is allowed and logged
as ideal; ROS all-zero covariance means unknown. This is not BNO055 calibration,
fusion, bias drift or dropout modeling. No GNSS, cameras, estimated odometry,
vehicle status, real command topic or speed loop is implemented.

All snapshots are evaluated at the post-step state, stamped with the same
MuJoCo time. Sensors/clock are published each physics step. A steady wall timer
requests `timestep / real_time_factor`; slow compute slows simulation instead of
skipping physics. Measured wall/simulation ratio is logged every five wall
seconds. Parameters are startup-only; `use_sim_time` cannot be set false.

## Simulation-only commands

This temporary interface **does not define `vehicle/command` or the future
physical actuator boundary**. `SimActuatorCommand` supplies a simulation timestamp,
current session ID, increasing sequence number, positive bounded validity and
one effort for every named motor. Hinge effort is Nm, slide effort is N. Only
stateless fixed-gain unit-gear joint motors are supported. Finite MJCF `ctrlrange`
limits must contain zero; out-of-range/nonfinite/partial/duplicate-name inputs
are rejected atomically, not clipped. Servos and other transmissions are rejected
at model load. Demo limits are synthetic test values, not kart specifications.

Future-dated, expired, wrong-session and duplicate/reordered commands are rejected.
Rejected commands neither replace a previous accepted command nor refresh its
deadline. Commands expire at `stamp + valid_for` or after `wall_timeout_s` since
acceptance, whichever is reached first. Checks occur before each physics step;
simulation expiry is quantized by at most one timestep. Expiry applies zero
**effort**, not a speed hold, brake or guarantee of stopping motion. A stalled
executor cannot advance physics; it clears expired effort before resuming.
Sequence history persists after expiry. Commands are never retained/replayed.

## Pause and reset

```bash
ros2 service call /kart/sim/pause std_srvs/srv/SetBool '{data: true}'
ros2 service call /kart/sim/reset std_srvs/srv/Trigger '{}'
ros2 service call /kart/sim/pause std_srvs/srv/SetBool '{data: false}'
```

Pause clears efforts, rotates the command session, and freezes physics; only
unchanged `/clock` heartbeats continue. Commands received while paused are
rejected. Reset requires pause, restores initial MJCF state and seed, clears
controls, rotates the session again, publishes t=0 and stays paused. Queued
pre-reset commands cannot match the new session. Restart also creates a new
session. Stop/reset downstream estimators/controllers before reset: this service
does not coordinate their queues/state or prevent stale sensor samples already
in transit. No production manager/arming session exists yet.

## Verification

From the repository root, with MuJoCo installed:

```bash
PYTHONPATH=simulation/src python -m unittest discover -s simulation/tests -v
PYTHONPATH=simulation/src:src/simulation/sim_ros python -m unittest discover -s src/simulation/sim_ros/tests -v
```

The second suite tests command validity without ROS and includes real ROS tests
for clock startup, message conversion, pause/reset and command delivery. ROS
tests explicitly skip when `rclpy` is absent. With ROS installed, build/source the
workspace first so generated interfaces are available; missing generated types
then fail rather than skip. macOS core tests do not validate DDS, ament, Linux
rendering, calibrated dynamics, Docker deployment or physical stopping.

Implementation references: [MuJoCo sensors](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-accelerometer),
[MuJoCo object velocity](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-objectvelocity),
and [Jazzy rclpy node/timer API](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/node.py).
