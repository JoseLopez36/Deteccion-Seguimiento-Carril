#!/usr/bin/env bash
set -euo pipefail

IMAGE="ghcr.io/autowarefoundation/autoware:core-humble"
CONTAINER_NAME="deteccion-seguimiento-carril-autoware-core"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

xhost +local:docker

docker run --rm -it \
  --name "${CONTAINER_NAME}" \
  --network host \
  --ipc host \
  --privileged \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -v "${WORKSPACE_DIR}:/workspace/deteccion-seguimiento-carril" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -w /workspace/deteccion-seguimiento-carril \
  "${IMAGE}" \
  bash
