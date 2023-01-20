#!/usr/bin/env bash
set -e

FILEPATH=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
cd "$FILEPATH"/.. || exit


sudo apt-get update -qq
sudo apt-get install -qq -y python3-vcstool git

# Install demo dependencies.
vcs import --recursive --skip-existing < "$FILEPATH"/my.repos
vcs pull

# Install mobipick's dependencies.
mobipick/install-deps.sh

# Install Unified Planning library and its planners.
pip install unified-planning==0.4.2.187.dev1
pip install up-fast-downward
pip install unified-planning-bridge/
