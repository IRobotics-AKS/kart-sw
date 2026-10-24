# ROS 2 structure and interface contract

Updated: 2026-10-24. Runtime contract targets for the monorepo. All 13 package
skeletons now have installation metadata; nodes and custom messages below remain
unimplemented. See [architecture.md](architecture.md).

## Packages and runtime responsibilities

| Package | Responsibility / expected runtime |
| --- | --- |
| `interfaces` | Custom trajectory, command, status, and services; no running node |
| `description` | URDF/Xacro, nominal dimensions, frames, RViz configuration; robot_state_publisher launched once |
| `sim_ros` | Simulation adapter: measurements/status/clock, command input, reset |
| `perception` | Perception integration and output normalization; selected vendor/Isaac nodes |
| `localization` | Sensor fusion and local/global pose estimation; reuse maintained estimators where appropriate |
| `planning` | Trajectory generation; initially a fixed-trajectory test publisher |
| `control` | Trajectory tracking and actuator-request generation |
| `sensor_drivers` | Driver integration/configuration and required adapters for OAK, BNO055, GNSS |
| `vehicle_bridge` | Jetson ROS to MCU protocol conversion; no planning or sensor fusion |
| `vehicle_manager` | Readiness, command source selection, arm/disarm coordination |
| `autonomy_bringup` | Launch autonomy, manager, and one robot description publisher |
| `sim_bringup` | Launch MuJoCo adapter and simulation configuration |
| `hardware_bringup` | Launch physical sensor integration and MCU bridge |

Each ROS package has `package.xml` and appropriate ament build/install metadata.
Use `ament_cmake` for ROS interface generation. Select C++ or Python per executable;
keep launch/configuration declarative and keep the control algorithm testable
outside ROS. Do not introduce fake package files without implementing installation.

The three bringup packages use explicit dependency sets. A deployment is composed
at the service layer; there is no mandatory all-components bringup dependency.
Firmware is outside the ROS build. Vendor repositories remain external dependencies.

## Namespaces and data sources

Use relative topic names in nodes and a configurable vehicle namespace, initially
`/kart`. Do not hardcode `/kart` in libraries. Fixed camera namespaces are
`front`, `side_left`, and `side_right`; bind each to a configured serial/MX ID,
not discovery order. Which physical side unit gets each ID remains to be measured.

Use exactly one measurement source and vehicle-command consumer per deployment:
physical drivers/bridge or simulation. `/clock` is the simulation time source.
For one-kart deployments use standard TF topics; explicit frame-ID prefixes and
TF remapping are required if multiple vehicles share a ROS graph. A ROS namespace
does not automatically prefix frame IDs.

| Relative topic (unless prefixed with /) | Type / contract | Live publisher | Simulation publisher |
| --- | --- | --- | --- |
| `sensors/imu/data` | `sensor_msgs/msg/Imu` | BNO055 integration | MuJoCo sensor model |
| `sensors/gnss/fix` | `sensor_msgs/msg/NavSatFix` | GNSS driver | MuJoCo GNSS model |
| `sensors/cameras/front/color/image_raw` | `sensor_msgs/msg/Image` | DepthAI driver | MuJoCo camera renderer |
| `sensors/cameras/front/color/camera_info` | `sensor_msgs/msg/CameraInfo` | DepthAI calibration | Matching virtual calibration |
| `sensors/cameras/side_left/color/image_raw` | `sensor_msgs/msg/Image` | DepthAI driver | MuJoCo camera renderer |
| `sensors/cameras/side_left/color/camera_info` | `sensor_msgs/msg/CameraInfo` | DepthAI calibration | Matching virtual calibration |
| `sensors/cameras/side_right/color/image_raw` | `sensor_msgs/msg/Image` | DepthAI driver | MuJoCo camera renderer |
| `sensors/cameras/side_right/color/camera_info` | `sensor_msgs/msg/CameraInfo` | DepthAI calibration | Matching virtual calibration |
| `vehicle/status` | Proposed `interfaces/msg/VehicleStatus` | MCU bridge | Simulated supervisor/actuators |
| `localization/odometry` | `nav_msgs/msg/Odometry` | Local estimator | Same estimator |
| `planning/trajectory` | Proposed `interfaces/msg/Trajectory` | Planner | Same planner or explicit test source |
| `control/request` | Proposed `interfaces/msg/ActuatorCommand` | Controller | Same controller |
| `vehicle/command` | Same actuator contract | Vehicle manager | Same vehicle manager |
| `sim/ground_truth/odometry` | `nav_msgs/msg/Odometry`, evaluation only | Absent | MuJoCo truth |
| `/clock` | `rosgraph_msgs/msg/Clock` | Absent | MuJoCo adapter |

Depth, stereo images, camera IMU, detections, magnetic-field data, and GNSS quality
status are enabled only when consumed and their exact contracts are defined. The
front OAK-D S2 can motivate depth/stereo pipelines; do not assign stereo depth to
an individual OAK-1 Lite or assume independent side cameras form a synchronized
stereo pair. Side-camera calibration and overlap are not yet established.

## Contract details before message implementation

- Trajectory: reference frame, start timestamp, ordered samples, pose/curvature
  representation, speed, timing, and expiration. A visualization path alone does
  not carry the complete control contract.
- ActuatorCommand: generation timestamp, session/sequence identifiers, bounded
  validity, operating intent, and explicit actuator fields/units. Final fields
  depend on whether the real boundary is current, torque, throttle/brake, or speed.
  Do not implement multiple ambiguous command modes without an explicit contract.
- VehicleStatus: MCU session/uptime, last accepted command sequence, arm/fault
  state, feedback validity, available measured steering/drive/brake feedback,
  active limiting, and diagnostics. Do not invent unavailable measurements.
- Distinguish measurement time from host reception time. Document cross-clock
  conversion for MCU/sensors; never treat MCU uptime as ROS epoch time.
- BNO055 fused orientation and gravity-compensated outputs require explicit
  estimator conventions and realistic covariance; they are not independent truth.
- GNSS must define geographic origin/altitude conventions and fix/correction
  quality. An RTK-capable kit does not guarantee continuous RTK-fixed measurements.
- Camera images, intrinsics, frame IDs, and synchronization must agree. Simulated
  perception inputs need realistic preprocessing and do not guarantee real-world
  visual performance.

Topics carry streams. Services handle bounded operations such as arm/disarm
requests and simulation reset. An accepted arm request is not proof of arming;
confirm the MCU's reported state. Actions may represent future long-running
missions. Continuous drive commands are not services/actions.

## Frames and publisher ownership

```text
map
  odom
    base_link
      imu_link
      gnss_link
      camera_front_link
        camera_front_color_optical_frame
      camera_side_left_link
        camera_side_left_color_optical_frame
      camera_side_right_link
        camera_side_right_color_optical_frame
      ... actual moving wheel/steering joints
```

- Local estimator owns `odom -> base_link`; global localization owns `map -> odom`.
  Disable competing TF output from other odometry sources.
- `autonomy_bringup` launches robot_state_publisher once; description and
  measured/simulated joint states provide internal transforms. Assign one owner
  for camera extrinsics; disable duplicate driver transforms if description owns them.
- Decide the physical `base_link` reference point from the kart geometry and use
  it consistently in control, simulation, and calibration.
- Use SI units and body X-forward/Y-left/Z-up; optical frames use Z-forward,
  X-right/Y-down. Convert vendor conventions at the adapter boundary.
- MuJoCo truth does not publish estimator-owned TF edges.

## Timing, QoS, and readiness

Simulation participants, including those on the Jetson, use `use_sim_time=true`.
Live nodes use real time. Simulation resets must clear old commands and relevant
queues, reset estimator/controller state, and invalidate prior arm sessions.
Document coordinated pause/reset behavior before relying on repeatable tests.
Ordinary distributed ROS communication is not deterministic lockstep stepping.

Start sensors with bounded queues and compatible sensor-data QoS where suitable.
Use reliable, volatile, shallow command queues as an initial configuration, with
explicit validity checks at the manager/bridge/MCU. Measure behavior under loss;
QoS alone does not enforce a motor stop or prove freshness. Do not replay retained
motion commands. Repeated stale commands must not refresh an MCU watchdog.

Choose concrete rates, queue depths, timeout budgets, and deadline thresholds
from measured compute/network/actuator behavior; these are not hardware-independent
constants. Track wall-clock latency and missed deadlines during Jetson tests.

The manager checks fresh required inputs, calibration/transforms, valid state,
selected source, and vehicle feedback. The MCU retains final physical arming and
watchdog authority. Startup and restart leave the vehicle disarmed. Missing inputs
or faults must lead to defined behavior; the final brake/stop policy awaits hardware.

## Launch interfaces

- `autonomy_bringup/autonomy.launch.py`: common stack; arguments include
  namespace, `use_sim_time`, configuration path, perception enablement, and
  planner versus explicit test-trajectory selection.
- `sim_bringup/simulation.launch.py`: model/scenario, sensor configuration,
  noise seed, timestep, rendering, and real-time pacing; no physical device path.
- `hardware_bringup/hardware.launch.py`: device mapping, calibration, GNSS
  correction settings, MCU transport, and live operation; reject simulation time
  for the physical actuator bridge.

The three launch files now exist and install through ament_cmake. They declare
their arguments for inspection with `ros2 launch ... --show-args`, but execution
raises a scaffold error before starting any process. Simulation requires
`use_sim_time=true`; hardware rejects it. Autonomy declares both clock modes.
No robot_state_publisher is launched until measured geometry is implemented.

Python runtime packages currently install namespaces and package resources with
no console scripts. `interfaces` is an ament_cmake/rosidl skeleton that
generates no types until reviewed interfaces are explicitly listed. The geometry
example is an unmeasured worksheet, not runtime configuration. See
[development.md](development.md) for build and validation instructions.

## References

- [ROS coordinate frames](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst)
- [ROS units and axes](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst)
- [ROS QoS](https://raw.githubusercontent.com/ros2/ros2_documentation/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)
- [ROS time](https://design.ros2.org/articles/clock_and_time.html)
