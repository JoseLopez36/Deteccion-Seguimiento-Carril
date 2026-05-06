#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch deteccion_seguimiento_carril_sim sim.launch.py