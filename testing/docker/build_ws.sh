#!/usr/bin/env bash
# Builds the mounted workspace inside the test container.
#
#   build_ws.sh                 build everything discoverable
#   build_ws.sh auv_controller  build just these packages (and their deps)
#   build_ws.sh --list-paths    print the package list and stop, for rosdep
#
# Two things make this more than a bare `colcon build`.
#
# 1. The workspace is whatever the person running it happens to have in
#    ~/ros2_ws/src, and some of it collides. Matsya_ROS2_Simulator ships
#    auv_worlds / auv_gui / auv_scoring / auv_simbridge under the same names as
#    the pinned simulator mounted at src/niot_simulator, and colcon refuses to
#    build a workspace with duplicate package names at all — it does not pick
#    one, it stops. So packages are discovered here, de-duplicated with the
#    pinned simulator winning, and handed to colcon as an explicit list.
#
# 2. The build lands in $TEST_WS_BASE, not in /ros2_ws/build and
#    /ros2_ws/install. Those belong to the host's native colcon build; mixing
#    container-built artefacts into them corrupts both.
set -euo pipefail

WS_SRC="${TEST_WS_SRC:-/ros2_ws/src}"
SIM_SRC="${TEST_SIM_SRC:-/ros2_ws/src/niot_simulator}"
BASE="${TEST_WS_BASE:-/opt/testws}"

# Matsya_ROS2_Simulator is the team's own copy of the simulator packages. It is
# skipped rather than preferred because the pinned tree in src/niot_simulator is
# the one this container promises to run. Override to swap that round, or to
# keep a half-finished checkout out of the build:
#
#   TEST_SKIP_DIRS="Matsya_ROS2_Simulator ASV_ROS2_Simulator" ./run.sh test_ws
TEST_SKIP_DIRS="${TEST_SKIP_DIRS:-Matsya_ROS2_Simulator}"

# ament_python packages are symlink-installed so that editing a node on the
# host takes effect on the next `ros2 run`, with no rebuild. Set false if a
# stale symlink farm ever needs to be ruled out.
TEST_SYMLINK="${TEST_SYMLINK:-true}"

# ROS's setup scripts read variables they have not necessarily set, so -u has to
# come off around them or sourcing dies on an unset AMENT_* / COLCON_*.
set +u
source /opt/ros/jazzy/setup.bash
set -u

declare -A provider=()
selected=()

# colcon prints paths relative to the cwd, so this runs from / and resolves
# them back to absolute. `colcon list` honours COLCON_IGNORE, which is what
# keeps the competition tree at src/NIOT_comp out of the build.
discover() {
  local base="$1" label="$2" name path
  [[ -d "$base" ]] || return 0
  while IFS=$'\t' read -r name path _; do
    [[ -n "$name" ]] || continue
    path="$(cd / && realpath "$path")"

    local skip=false dir
    for dir in $TEST_SKIP_DIRS; do
      [[ "$path" == *"/$dir/"* ]] && skip=true
    done
    if $skip; then
      printf '  skip     %-24s %s (TEST_SKIP_DIRS)\n' "$name" "$path" >&2
      continue
    fi

    # The pinned simulator is mounted *inside* the workspace's src, so the
    # second crawl finds it again. Same path, nothing to report.
    if [[ "${provider[$name]:-}" == "$path" ]]; then
      continue
    fi

    if [[ -n "${provider[$name]:-}" ]]; then
      printf '  shadowed %-24s %s\n' "$name" "$path" >&2
      printf '           %-24s (using %s)\n' "" "${provider[$name]}" >&2
      continue
    fi

    provider[$name]="$path"
    selected+=("$path")
    printf '  %-8s %-24s %s\n' "$label" "$name" "$path" >&2
  # --log-base /dev/null because `colcon list` otherwise insists on creating a
  # log/ directory in the working directory, which here is /.
  done < <(cd / && colcon --log-base /dev/null list --base-paths "$base" 2>/dev/null)
}

echo "Discovering packages" >&2
# Order matters: the first path to claim a package name keeps it, so the pinned
# simulator is crawled before the rest of the workspace.
discover "$SIM_SRC" "sim"
discover "$WS_SRC"  "ws"

if [[ ${#selected[@]} -eq 0 ]]; then
  echo "No packages found under $WS_SRC." >&2
  echo "Is the workspace mounted? Check NIOT_WS in ./run.sh test_up." >&2
  exit 1
fi

# ./run.sh test_deps wants exactly this list: rosdep crawls directories rather
# than packages, so pointed at the mounted workspace it would walk straight into
# the same duplicate package names colcon refuses to build.
if [[ "${1:-}" == "--list-paths" ]]; then
  printf '%s\n' "${selected[@]}"
  exit 0
fi

echo
echo "Building ${#selected[@]} packages into $BASE"

extra_args=()
[[ "$TEST_SYMLINK" == "true" ]] && extra_args+=(--symlink-install)

# Named packages are built with their dependencies, which is nearly always what
# someone typing `./run.sh test_ws auv_controller` means.
[[ $# -gt 0 ]] && extra_args+=(--packages-up-to "$@")

# --continue-on-error, and deliberately: this workspace holds whatever a
# teammate is halfway through. One package that does not compile should cost
# them that package, not the simulator they were trying to test against.
set +e
# --log-base is a global colcon option, before the verb, not a build option.
colcon --log-base "$BASE/log" build \
  --paths "${selected[@]}" \
  --build-base "$BASE/build" \
  --install-base "$BASE/install" \
  --continue-on-error \
  --event-handlers console_cohesion+ \
  "${extra_args[@]}"
build_status=$?
set -e

# The build is allowed to be partially broken; the simulator is not. These
# three are what `./run.sh test_up` is about to launch, so a missing one is
# reported as the failure it is rather than surfacing later as a launch error
# about a package that does not exist.
missing=()
for pkg in auv_msgs auv_worlds auv_simbridge; do
  [[ -d "$BASE/install/$pkg" ]] || missing+=("$pkg")
done

echo
if [[ ${#missing[@]} -gt 0 ]]; then
  echo "FAILED: ${missing[*]} did not build — the simulator cannot start." >&2
  echo "Full logs: $BASE/log/latest_build" >&2
  exit 1
fi

if [[ $build_status -ne 0 ]]; then
  echo "Workspace built with failures. The simulator packages are fine, so"
  echo "./run.sh test_up will work; the failed packages are in $BASE/log/latest_build."
else
  echo "Workspace built clean."
fi
