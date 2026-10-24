# Build-only scaffold recipe. Supply a validated ROS Jazzy / Python 3.12 base.
ARG ROS_BASE_IMAGE
FROM ${ROS_BASE_IMAGE}

SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=jazzy \
    VIRTUAL_ENV=/opt/kart/venv \
    PATH=/opt/kart/venv/bin:$PATH \
    ROS_LOG_DIR=/tmp/ros/log
WORKDIR /opt/kart

RUN test -f /opt/ros/jazzy/setup.bash \
    && python3 -c 'import sys; assert sys.version_info[:2] == (3, 12)' \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
       python3-colcon-common-extensions python3-rosdep python3-venv \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m venv --system-site-packages /opt/kart/venv
COPY --chmod=755 deploy/docker/entrypoint.sh /usr/local/bin/kart-entrypoint
COPY --chmod=755 deploy/docker/build-workspace.sh /usr/local/bin/kart-build-workspace
COPY simulation/sim /opt/kart/simulation/sim
RUN python -m pip install --no-cache-dir /opt/kart/simulation/sim
COPY src/common/interfaces /opt/kart/src/common/interfaces
COPY src/simulation/sim_ros /opt/kart/src/simulation/sim_ros
COPY src/bringup/sim_bringup /opt/kart/src/bringup/sim_bringup
RUN kart-build-workspace sim_bringup \
    && useradd --uid 10001 --user-group --create-home kart
USER kart
ENTRYPOINT ["kart-entrypoint"]
CMD ["ros2", "launch", "sim_bringup", "simulation.launch.py"]
