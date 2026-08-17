#!/usr/bin/env bash
# Sources ROS and the built workspace, then hands off to the command.
#
# Also sourced from ~/.bashrc with --shell-init so that `docker exec ... bash`
# lands in an environment where `ros2 run` works, which it otherwise would not:
# exec bypasses the entrypoint.
#
# Deliberately no `set -e`: this file gets sourced into interactive shells,
# where it would turn any failed command into a closed terminal.

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

[[ "$1" == "--shell-init" ]] && return 0

exec "$@"
