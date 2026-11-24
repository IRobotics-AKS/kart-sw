# Container images

Four separate build recipes use the repository root as context. Each copies only
its package dependency set before rosdep and colcon run. The simulation recipe
explicitly installs the standalone Python package and its packaged demo MJCF.

The workstation simulation recipe defaults to a verified multi-architecture ROS
base digest and runs the explicit headless demo. See [the runbook](../README.md).
The other recipes still require `--build-arg ROS_BASE_IMAGE=...`. Supply an inspected ROS Jazzy
base with Ubuntu 24.04/Python 3.12 and pin its digest for releases. No NVIDIA tag,
registry, JetPack compatibility, GPU runtime, or vendor driver pin is invented here.
The simulation Python project now declares MuJoCo 3.16.0, which its image build
installs. Workstation container verification is recorded in
[deployment docs](../../docs/docker-deployment.md); no Isaac ROS pipeline is included.

```sh
docker build -f deploy/docker/autonomy.Dockerfile \
  --build-arg ROS_BASE_IMAGE="$ROS_BASE_IMAGE" \
  -t kart-autonomy:scaffold .
```

Use the matching filename/tag for simulation, hardware, or recorder. These are
development recipes: build tools remain in the image, apt/rosdep/build-tool
dependencies are not locked, and no release image has been validated. Move to
pinned multi-stage release builds after the first implemented runtime is tested.

Entrypoints source ROS/the installed overlay and exec the supplied command.
They do not build, download, arm, or flash on startup. The runtime user is
unprivileged; physical-device groups and mappings are intentionally absent.
Autonomy and hardware bringup commands still fail with a clear scaffold error. Recorder defaults
to help; explicit topics/storage budgets are needed before recording.
