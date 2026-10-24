#!/usr/bin/env bash
# Only the service-specific dependency closure is copied into /opt/kart/src.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    rosdep init
fi
rosdep update --rosdistro jazzy
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
# Run colcon with the same Python environment as the standalone simulation core.
python "$(command -v colcon)" build --base-paths src --packages-up-to "$1" \
    --event-handlers console_direct+
