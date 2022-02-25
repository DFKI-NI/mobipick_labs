#!/usr/bin/env bash
export FILEPATH=$(dirname `realpath "${BASH_SOURCE[0]}"`)
cd $FILEPATH/../..

# Install dependencies.
vcs import < $FILEPATH/my.repos

# Also install mobipick's dependencies.
mobipick/install-deps.sh

# Install Unified Planning library and Pyperplan planner.
pip install unified-planning/
pip install up-pyperplan/

# Checkout working commit in geometric_shapes repository as workaround.
cd geometric_shapes/
git checkout ca019f4
