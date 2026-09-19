# Sources the ROS distro, the third-party underlay and, once built, the
# workspace overlay into the current shell.
#
# docker/entrypoint.sh does the same for the container's main process only; a
# shell opened later with `docker exec` or a VS Code terminal inherits none of
# it. Source this file there: `source /ws/setup.sh`.

source /opt/ros/jazzy/setup.bash

if [ -f /opt/racing_underlay/setup.bash ]; then
    source /opt/racing_underlay/setup.bash
fi

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi
