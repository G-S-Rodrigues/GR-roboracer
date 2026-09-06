#!/usr/bin/env bash
# Sources the ROS distro, the third-party underlay, and -- if it has been built
# -- the workspace overlay, then execs the command.
#
# The overlay is sourced only when install/setup.bash exists, so a fresh clone
# with no build yet still gets a usable shell instead of an error.
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /opt/racing_underlay/setup.bash ]; then
    source /opt/racing_underlay/setup.bash
fi

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
