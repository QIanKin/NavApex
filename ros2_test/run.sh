#!/usr/bin/env bash
set -eo pipefail

navapex_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$navapex_root"
source /opt/ros/humble/setup.bash

CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 nice -n 10 /usr/bin/colcon --log-base ros2_test/log build \
    --base-paths ros2_test --build-base ros2_test/build --install-base ros2_test/install \
    --executor sequential --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3

source ros2_test/install/setup.bash
exec ros2 launch navapex_ros2 simulation.launch.py gui:=true rviz:=true "$@"
