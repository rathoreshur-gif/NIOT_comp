#!/usr/bin/env bash
# Sources ROS and the container-built workspace, then hands off to the command.
#
# Also sourced from ~/.bashrc with --shell-init so that `docker exec ... bash`
# lands in an environment where `ros2 run` works, which it otherwise would not:
# exec bypasses the entrypoint.
#
# Deliberately no `set -e`: this file gets sourced into interactive shells,
# where it would turn any failed command into a closed terminal.

TEST_WS_BASE="${TEST_WS_BASE:-/opt/testws}"

source /opt/ros/jazzy/setup.bash

# Unlike the competition image, the workspace here is built at *runtime* from
# the mounted sources, so on the very first start there is nothing to source
# yet. build_ws.sh creates it; until then this is a bare ROS environment and
# saying so beats a confusing "no such file" from a sourced script.
if [[ -f "$TEST_WS_BASE/install/setup.bash" ]]; then
  source "$TEST_WS_BASE/install/setup.bash"
else
  echo "note: no workspace built yet — run ./run.sh test_ws (or build_ws.sh in here)" >&2
fi

# competition.launch.py appends these itself, but the generated world lands in
# a temp dir outside the share tree, so seeding them here keeps model:// URIs
# and the Scoreboard/AuvTeleop plugins resolvable for anything run by hand too.
export GZ_SIM_RESOURCE_PATH="$TEST_WS_BASE/install/auv_worlds/share/auv_worlds/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_GUI_PLUGIN_PATH="$TEST_WS_BASE/install/auv_gui/lib/auv_gui${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"

[[ "$1" == "--shell-init" ]] && return 0

exec "$@"
