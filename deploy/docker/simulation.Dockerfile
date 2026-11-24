# CPU workstation image, not a Jetson/JetPack image. Multi-architecture index verified 2026-11-24.
ARG ROS_BASE_IMAGE=ros:jazzy-ros-base-noble@sha256:5147d3ae250ce754afd399f6bfec15b33dd4e473ca357e30432a3182031bfad6
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=jazzy \
    VIRTUAL_ENV=/opt/kart/venv \
    PATH=/opt/kart/venv/bin:$PATH \
    ROS_LOG_DIR=/tmp/ros/log \
    ROS_HOME=/tmp/ros \
    PYTHONDONTWRITEBYTECODE=1
WORKDIR /opt/kart

RUN test -f /opt/ros/jazzy/setup.bash \
    && python3 -c 'import sys; assert sys.version_info[:2] == (3, 12)' \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-venv \
       libgl1 libegl1 libglfw3 ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv --system-site-packages /opt/kart/venv
COPY --chmod=755 deploy/docker/entrypoint.sh /usr/local/bin/kart-entrypoint
COPY --chmod=755 deploy/docker/build-workspace.sh /usr/local/bin/kart-build-workspace
COPY simulation/ /opt/kart/simulation/
RUN python -m pip install --no-cache-dir /opt/kart/simulation
COPY src/common/interfaces /opt/kart/src/common/interfaces
COPY src/simulation/sim_ros /opt/kart/src/simulation/sim_ros
COPY src/bringup/sim_bringup /opt/kart/src/bringup/sim_bringup
RUN kart-build-workspace sim_bringup \
    && useradd --uid 10001 --user-group --create-home kart
COPY tools/sim_docker_smoke.py /opt/kart/tools/sim_docker_smoke.py
COPY --chmod=755 deploy/docker/test-simulation.sh /usr/local/bin/kart-test-simulation
USER kart
ENTRYPOINT ["kart-entrypoint"]
CMD ["ros2", "launch", "sim_bringup", "simulation.launch.py", "model:=demo", "config_file:=/opt/kart/install/sim_bringup/share/sim_bringup/config/bridge_demo.yaml", "rendering:=false"]
