# Docker deployment design

Updated: 2026-10-24. This document specifies the target deployment. Dockerfiles
and Compose templates now exist and pass static/configuration checks. No images
have been built, runtime deployments validated, or boot units implemented.
See [architecture.md](architecture.md) and [ROS structure](ros-structure.md).

## Host and image responsibilities

The Jetson host provides its supported JetPack/Linux kernel, NVIDIA drivers and
container runtime, networking, udev/device setup, and power/cooling configuration.
The containers provide compatible ROS/userspace libraries and installed project
packages. Containers do not replace the Jetson kernel or provide real-time safety.
MCU and VESC firmware run on their devices, outside Docker.

Use native Linux for full integration. A workstation without a supported NVIDIA
GPU can run suitable CPU-only control/localization tests, but cannot be assumed
to run Isaac ROS GPU perception. macOS can run native MuJoCo development; Docker
Desktop is not equivalent to the Jetson GPU/device environment.

## Images and services

| Service | Packages / contents | Host/device needs |
| --- | --- | --- |
| `simulation` | Standalone MuJoCo library/assets; interfaces; sim adapter; sim bringup | Workstation; rendering backend as configured; no actuator devices |
| `autonomy` | Interfaces, description, perception, localization, planning, control, manager, autonomy bringup | Workstation or Jetson; NVIDIA runtime/GPU if accelerated perception is enabled |
| `hardware` | Interfaces, vendor sensor drivers/integration, MCU bridge, hardware bringup | Jetson; only required USB/serial/I2C/CAN access according to actual wiring |
| `recorder` (optional) | ROS bag tooling and custom message definitions | Either host; persistent bounded recording storage |

The OAK devices may run compatible inference onboard. Start with Jetson-side
perception or an explicitly selected OAK pipeline and benchmark before splitting
high-rate processing into more containers. Coral and BRIO are not image dependencies.
The simulator and hardware both implement the same ROS measurement/status/command
boundary; they are not started together for one kart deployment.

## Target files

```text
deploy/
  docker/
    simulation.Dockerfile
    autonomy.Dockerfile
    hardware.Dockerfile
    recorder.Dockerfile
  compose/
    workstation-sim.yaml
    simulator-server.yaml
    jetson-sim.yaml
    kart-live.yaml
    development.override.yaml
  config/
    middleware/                  DDS configuration per validated transport
    releases/                    Source commit, image digests, firmware/config versions
  systemd/
    kart.service
```

Use root build context with an explicit `.dockerignore` and selective COPY steps.
Exclude `.git`, virtual environments, build/install/log trees, datasets, recordings,
secrets, and historical handoff files from build context. Copy only each image's
source/dependencies in the final image; compiler and development tools can remain
in separate build stages or a development target.

## Operating-mode composition

| Compose file | Host | Services | Clock / command destination |
| --- | --- | --- | --- |
| `workstation-sim.yaml` | Workstation | simulation, autonomy, optional recorder | MuJoCo `/clock`; commands to MuJoCo |
| `simulator-server.yaml` | Workstation | simulation, optional recorder | Publishes `/clock`; consumes Jetson commands |
| `jetson-sim.yaml` | Jetson | autonomy, optional recorder | Subscribes to workstation `/clock`; commands to MuJoCo |
| `kart-live.yaml` | Jetson | autonomy, hardware, optional recorder | Real time; commands to MCU |

For remote simulation run simulator-server and jetson-sim separately on their
respective hosts. Standard Compose does not schedule a deployment across hosts.
Use the same source revision and compatible configuration on both. Do not rely on
cross-host `depends_on`; autonomy waits for valid inputs and remains inactive
until they arrive. A network interruption must invalidate commands appropriately.

Use separate deployment files, not an easily mis-set single `real_hardware` flag.
Simulation files must not include the hardware service. Enforce an operational
single-mode lock/selection on the Jetson to avoid a stale live deployment running
alongside a simulation deployment. Starting simulation does not automatically stop
an independently launched live stack. Preserve physical disarming during tests.

## Builds and pins

Build `amd64` workstation and `arm64` Jetson variants from the same Git revision
where needed, using appropriate base images. Not every architecture supports
identical GPU libraries. A CPU-only test variant may omit optional perception but
must not be claimed to validate the complete GPU pipeline.

Use the selected ROS distribution consistently. Pin MuJoCo and other Python
libraries. Record NVIDIA base image/runtime compatibility and use image digests
for release deployment. Do not invent registry URLs or unverified NVIDIA image
tags. Record firmware revision and relevant configuration/calibration versions
alongside images; one Git commit does not capture externally changed device state.

The simulation image installs `simulation` and its assets explicitly,
then builds `sim_bringup` and its ROS dependencies. Autonomy and hardware
images build their own bringup packages and recursive dependencies. Example build
selection for the current package skeletons:

```sh
colcon build --base-paths src --packages-up-to autonomy_bringup
```

See [colcon package selection](https://colcon.readthedocs.io/en/released/reference/package-selection-arguments.html).
Dependency installation must also be scoped; installing dependencies for every
package in the repo can reintroduce MuJoCo/drivers into an unrelated image.

## Networking and device access

For initial native-Linux integration use host networking with a documented DDS
implementation/configuration, correct network interface selection, and explicit
firewall/discovery settings. No port mapping is needed with host networking.
Verify cross-container transport/shared-memory behavior; if necessary configure
UDP-only transport initially rather than assuming shared IPC works automatically.
Do not enable host IPC or privileged mode without an identified requirement.

Use the same ROS domain within one deployment and distinct domains for independent
live/simulation tests. Domain IDs are discovery separation, not authorization.
Host networking shares interfaces, including potential CAN interfaces; Docker
service separation alone is not a physical actuation security boundary. Restrict
capabilities/network access as needed and keep the physical MCU disarmed in
simulation. Only deploy the real bridge in the live configuration.

Use stable device identities: OAK serial/MX IDs, persistent serial symlinks, and
explicit CAN interface configuration. Do not bind roles to USB enumeration order.
Some USB devices re-enumerate during initialization; validate required device
mappings/udev rules with actual hardware. Decide BNO055 and GNSS transport from
wiring rather than assuming every sensor is USB. Keep GNSS correction credentials
outside Git and images.

## Runtime configuration and recovery

Mount calibration and runtime configuration read-only. Write bags/logs to
persistent storage with retention and resource limits. Do not compile, download
packages, or flash firmware on routine container startup.

Use ROS launch to manage the nodes inside a service and arrange for critical
child-process failure to be visible to supervision. A container can remain alive
when a critical node has exited. Health checks assess readiness and data freshness,
not just process existence. Compose health checks can gate startup, but an unhealthy
flag by itself does not automatically restart a container.

Set bounded shutdown behavior so launch receives termination signals. Define
restart policies explicitly; a restart invalidates the prior control session and
must not rearm. Optional systemd boot integration starts only the selected live
Compose deployment after host prerequisites; it starts software, not motion.
The manager and MCU enforce readiness/arming independently of container recovery.

## Validation before deployment

- Validate rendered Compose configuration and ensure each mode has the expected
  services, image architecture, clock configuration, and device access.
- Build clean images and smoke-test each installed launch/package entry point.
- Verify sensor and TF ownership, compatible QoS, and the command destination.
- Test command expiration, disconnect/reconnect, child-node failure, reset/time
  jumps, and prevention of automatic rearming.
- Benchmark Jetson RAM, CPU/GPU, USB bandwidth, end-to-end latency, and storage I/O
  with all selected camera streams. Docker does not create extra hardware capacity.
- Verify physical stop behavior separately on the actual system under the agreed
  test procedure. A successful MuJoCo run does not establish real stopping behavior.

## References

- [Isaac ROS 4.6 environment and Docker setup](https://nvidia-isaac-ros.github.io/v/release-4.6/getting_started/index.html)
- [Compose startup and health checks](https://docs.docker.com/compose/how-tos/startup-order/)
- [Docker host networking](https://docs.docker.com/engine/network/drivers/host/)
