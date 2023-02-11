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
pip3 install 'unified-planning[tamer,fast-downward]==0.5.0.34.dev1'
pip3 install 'pyparsing>=3'   # has to be installed manually, see https://github.com/aiplan4eu/unified-planning/issues/325
pip3 install unified-planning-bridge/
