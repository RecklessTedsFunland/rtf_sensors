#!/bin/bash
# -----------------------------------------
# MIT License, Kevin J. Walchko (c) 2020
# ./build.sh [package]
#

# exit on error
set -e

if [[ -d "src" ]]; then
    if [[ $# -eq 1 ]]; then
        colcon build --symlink-install --packages-select $1 && . install/setup.bash
    else
        colcon build --symlink-install && . install/setup.bash
    fi
fi

