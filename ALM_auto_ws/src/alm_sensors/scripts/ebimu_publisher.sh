#!/usr/bin/env bash
set -e

EBIMU_SETUP="${EBIMU_SETUP:-/home/dong/ebimu_ws/install/setup.bash}"

if [ ! -f "$EBIMU_SETUP" ]; then
  echo "EBIMU setup file not found: $EBIMU_SETUP" >&2
  exit 1
fi

source "$EBIMU_SETUP"
exec ros2 run ebimu_pkg ebimu_publisher
