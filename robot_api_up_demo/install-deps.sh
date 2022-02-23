#!/usr/bin/env bash
set -e -x
export ROS_DISTRO="noetic"
export FILEPATH=$(dirname `realpath "${BASH_SOURCE[0]}"`)
cd $FILEPATH/../..

# Install dependencies.
sudo apt update -qq
sudo apt install -qq -y python3-wstool git
if [ ! -f .rosinstall ]; then
  wstool init
fi
wstool merge --merge-keep -y $FILEPATH/dependencies.rosinstall
wstool update

# Also install mobipick's dependencies.
mobipick/install-deps.sh

# Install Unified Planning library and Pyperplan planner.
pip install unified-planning/
pip install up-pyperplan/

# Checkout working commit in geometric_shapes repository as workaround.
cd geometric_shapes/
git checkout ca019f4
