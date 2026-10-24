# Image scaffold

Four separate build recipes use the repository root as context. Each copies only
its package dependency set before rosdep and colcon run. The simulation recipe
explicitly installs the standalone Python package and its current asset notes.

All recipes require `--build-arg ROS_BASE_IMAGE=...`. Supply an inspected ROS Jazzy
base with Ubuntu 24.04/Python 3.12 and pin its digest for releases. No NVIDIA tag,
registry, JetPack compatibility, GPU runtime, or vendor driver pin is invented here.
The current scaffold contains no MuJoCo runtime dependency or Isaac ROS pipeline.

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
Bringup commands currently fail with a clear scaffold error. Recorder defaults
to help; explicit topics/storage budgets are needed before recording.
