#!/usr/bin/env bash
set -euo pipefail
cd /opt/kart
# Fail instead of silently skipping ROS tests in a broken image.
python -c 'import rclpy; from interfaces.msg import SimActuatorCommand'
python -m unittest discover -s simulation/tests -v
python -m unittest discover -s src/simulation/sim_ros/tests -v
# Exercise installed package assets outside the checkout.
cd /tmp
python -m sim --model demo --duration 0.1
