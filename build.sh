#!/usr/bin/env bash
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"/../..
source /opt/ros/noetic/setup.bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
catkin build
