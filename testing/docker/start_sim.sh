#!/usr/bin/env bash
# What the test container runs as its command: build the workspace if it has
# not been built yet, then start the simulator with the simbridge attached.
#
# The build is here rather than in the image because the sources arrive as a
# bind mount that does not exist at image build time. It is skipped on every
# start after the first, so `./run.sh test_down && ./run.sh test_up` is quick;
# `./run.sh test_ws` forces it, and so does TEST_REBUILD=1.
set -euo pipefail

BASE="${TEST_WS_BASE:-/opt/testws}"

if [[ "${TEST_REBUILD:-0}" == "1" || ! -d "$BASE/install/auv_worlds" ]]; then
  echo "=== Building the workspace (first start, or TEST_REBUILD=1) ==="
  /usr/local/bin/build_ws.sh
  echo
fi

# -u off around the setup scripts: they read AMENT_* / COLCON_* variables they
# have not necessarily set, and would abort the container on start.
set +u
source "$BASE/install/setup.bash"
set -u

# Same environment the entrypoint seeds, re-applied because the install tree
# only exists once the build above has run.
export GZ_SIM_RESOURCE_PATH="$BASE/install/auv_worlds/share/auv_worlds/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_GUI_PLUGIN_PATH="$BASE/install/auv_gui/lib/auv_gui${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"

# scoring defaults OFF here, unlike the competition stack: this container is for
# driving the vehicle around and testing the team's own nodes against it, and
# the flaggers are noise in that setting. SIM_SCORING=true turns them back on.
#
# teleop defaults OFF for the reason the competition stack forces it off: the
# teleop node publishes to the same /model/auv/joint/*/cmd_thrust topics as
# simulator_bridge, at 50 Hz, and freezes position/heading/depth whenever no key
# is held — so left on it silently station-keeps against whatever controller is
# being tested. SIM_TELEOP=true only when driving by hand with nothing else up.
echo "=== Starting simulator + simbridge ==="
exec ros2 launch auv_worlds competition.launch.py \
  simulator_bridge:=true \
  gui:="${SIM_GUI:-true}" \
  seed:="${SIM_SEED:-auto}" \
  scoring:="${SIM_SCORING:-false}" \
  teleop:="${SIM_TELEOP:-false}" \
  paused:="${SIM_PAUSED:-false}" \
  current:="${SIM_CURRENT:-true}"
