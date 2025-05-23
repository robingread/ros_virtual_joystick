#! /bin/bash

set -e

colcon clean workspace --yes
colcon build
colcon test
colcon test-result --verbose --all
