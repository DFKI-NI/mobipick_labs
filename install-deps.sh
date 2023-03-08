#!/usr/bin/env bash
set -e

FILEPATH=$(dirname "$(realpath "${BASH_SOURCE[0]}")")
cd "$FILEPATH"/.. || exit


sudo apt-get update -qq
sudo apt-get install -qq -y python3-vcstool python3-pip git

# Install demo dependencies.
vcs import --recursive --skip-existing < "$FILEPATH"/my.repos
vcs pull

# Install mobipick's dependencies.
mobipick/install-deps.sh

# Install Unified Planning library and its planners.
pip3 install 'unified-planning[tamer,fast-downward]==0.5.0.304.dev1'
pip3 install unified-planning-bridge/
