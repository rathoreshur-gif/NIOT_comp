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

# competition.launch.py appends these itself, but the generated world lands in
# a temp dir outside the share tree, so seeding them here keeps model:// URIs
# and the Scoreboard/AuvTeleop plugins resolvable for anything run by hand too.
export GZ_SIM_RESOURCE_PATH="/ros2_ws/install/auv_worlds/share/auv_worlds/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_GUI_PLUGIN_PATH="/ros2_ws/install/auv_gui/lib/auv_gui${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"

[[ "$1" == "--shell-init" ]] && return 0

exec "$@"
