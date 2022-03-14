#!/usr/bin/env bash
FILEPATH=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
cd "$FILEPATH"/.. || exit


sudo apt-get update -qq
sudo apt-get install -qq -y python3-vcstool git

# First install mobipick's dependencies.
mobipick/install-deps.sh

# Then install demo dependencies.
vcs import < "$FILEPATH"/my.repos

# Install Unified Planning library and its planners.
pip install unified-planning/
pip install up-pyperplan/
pip install up-tamer/

# Checkout working commit in geometric_shapes repository as workaround.
cd geometric_shapes/ || exit
git checkout ca019f4
