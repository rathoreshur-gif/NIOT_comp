#!/usr/bin/env bash
# Thin wrapper around docker compose that deals with the X11 handshake and the
# NVIDIA runtime, both of which are easy to forget and fail in confusing ways.
#
#   ./run.sh build         build (or rebuild) both images
#   ./run.sh up            start sim + domain_bridge + competitor
#   ./run.sh sim           follow the simulator's logs
#   ./run.sh bridge        follow the domain bridge's logs
#   ./run.sh controller    follow the competitor controller's logs
#   ./run.sh shell [svc]   open a shell in a service (default: sim)
#   ./run.sh topics [svc]  list the topics that service can see
#   ./run.sh gpu           report whether the GPU overlay is in use, and why
#   ./run.sh down          stop and remove everything
#
# The team's testing container is a separate stack, sharing none of the above.
# It runs the whole workspace — the simulator and the Matsya ROS 2 code — in one
# container, for ordinary development rather than for a competition run:
#
#   ./run.sh test_build    build (or rebuild) the testing image
#   ./run.sh test_up       start the simulator + simbridge on the mounted ws
#   ./run.sh test_shell    a shell in it, workspace sourced
#   ./run.sh test_logs     follow what the simulator is saying
#   ./run.sh test_ws [pkg] rebuild the workspace inside the container
#   ./run.sh test_topics   list the topics it can see
#   ./run.sh test_sync     re-export the pinned simulator tree
#   ./run.sh test_deps     rosdep the mounted workspace (until the next rebuild)
#   ./run.sh test_pip      pip install Matsya_ROS2/requirements.txt in there
#   ./run.sh test_down     stop and remove it
#
# See TESTING.md. Anything else is passed through to `docker compose` verbatim.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

# Gazebo's GUI is an X client; without this the container's connection to the
# host display is refused. Harmless to re-run, and scoped to local connections.
grant_x11() {
  command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
}

# Gazebo renders the camera sensors on the GPU, so the NVIDIA overlay is worth
# real RTF. It cannot be on by default: `runtime: nvidia` makes the container
# fail to start on a machine without the toolkit, which is most competitors.
#
# The test is for the *binary*, not `docker info`. Docker keeps a stale `nvidia`
# runtime registration on machines where the toolkit was never installed or was
# removed, so asking Docker what runtimes it knows about reports a runtime that
# does not work.
gpu_wanted() {
  case "${NIOT_GPU:-auto}" in
    0|false|off|no) return 1 ;;
    1|true|on|yes)  return 0 ;;
    *) command -v nvidia-container-runtime >/dev/null 2>&1 ;;
  esac
}

# Explicit -f arguments disable Compose's automatic pickup of
# docker-compose.override.yml, so it has to be listed again by hand. Last file
# wins, which is why the user's override goes at the end.
COMPOSE_FILES=(-f docker-compose.yml)
gpu_wanted && COMPOSE_FILES+=(-f docker-compose.gpu.yml)
[[ -f docker-compose.override.yml ]] && COMPOSE_FILES+=(-f docker-compose.override.yml)

# `docker compose exec` bypasses the ENTRYPOINT, so a non-interactive command
# would run without ROS on its path. Calling the entrypoint explicitly restores
# it. Interactive shells get the same treatment via ~/.bashrc.
in_container() {
  exec docker compose "${COMPOSE_FILES[@]}" exec "$1" /entrypoint.sh "${@:2}"
}

ensure_up() { grant_x11; docker compose "${COMPOSE_FILES[@]}" up -d; }

# ---------------------------------------------------------------------------
# The testing stack (./run.sh test_*). Its own Compose project, so that
# `./run.sh down` on the competition stack cannot reach into it and vice versa.
# ---------------------------------------------------------------------------

TEST_FILES=(-p niot-test -f docker-compose.test.yml)
gpu_wanted && TEST_FILES+=(-f docker-compose.test.gpu.yml)
[[ -f docker-compose.test.override.yml ]] && TEST_FILES+=(-f docker-compose.test.override.yml)

# The simulator the testing container runs is exported from a pinned commit, not
# taken from the working tree, and that is the point rather than an oversight:
# the working tree carries the demo build — the gamepad path, the START/END run
# lifecycle, the payload models, FollowCam — which exists for showing the
# simulator at a stand and gets in the way of testing a controller against it.
# This ref is the competition release as it stood before any of that landed.
#
#   NIOT_TEST_REF=<ref> ./run.sh test_sync    to track something else
NIOT_TEST_REF="${NIOT_TEST_REF:-56164bd}"

# Where the workspace being tested lives. Defaults to the parent of this repo's
# parent — ros2_ws/src/NIOT_comp puts it at ros2_ws — which is where a normal
# checkout sits. NIOT_WS overrides it for anyone who keeps theirs elsewhere.
test_ws_root() {
  local ws
  ws="${NIOT_WS:-$(cd ../.. && pwd)}"
  if [[ ! -d "$ws/src" ]]; then
    echo "No colcon workspace at $ws (expected $ws/src)." >&2
    echo "Set NIOT_WS to your workspace root:  NIOT_WS=~/ros2_ws ./run.sh test_up" >&2
    exit 1
  fi
  printf '%s' "$ws"
}

# auv_msgs is vendored into simulator/ so the competition repo builds on its own,
# but the team workspace has its own copy in Matsya_ROS2 — the same package name,
# and colcon refuses to build a workspace containing both. The workspace's copy
# wins when there is one, because that is the one the team's nodes are written
# against; the vendored copy is exported only as a fallback for a workspace
# without it. NIOT_comp itself is skipped: its tree is COLCON_IGNOREd.
ws_provides_auv_msgs() {
  find "$1/src" -maxdepth 3 -name package.xml -not -path "*/NIOT_comp/*" -print0 2>/dev/null \
    | xargs -0 grep -l "<name>auv_msgs</name>" 2>/dev/null | grep -q .
}

# Idempotent: re-exports only when the pinned ref moved, so a `test_up` does not
# rewrite the tree and make colcon rebuild the simulator every single time.
sync_sim() {
  local sha stamp paths
  git rev-parse --git-dir >/dev/null 2>&1 || {
    echo "Not a git checkout, so the pinned simulator cannot be exported." >&2
    echo "Clone this repository rather than copying it, or point" >&2
    echo "NIOT_TEST_REF at a checkout that has history." >&2
    exit 1
  }
  sha="$(git rev-parse --verify "${NIOT_TEST_REF}^{commit}" 2>/dev/null)" || {
    echo "NIOT_TEST_REF=$NIOT_TEST_REF does not resolve to a commit." >&2
    exit 1
  }

  stamp=".niot-test/simulator/.exported-from"
  if [[ "${1:-}" != "--force" && -f "$stamp" && "$(cat "$stamp")" == "$sha" ]]; then
    return 0
  fi

  echo "Exporting the simulator from ${NIOT_TEST_REF} (${sha:0:7})"
  rm -rf .niot-test/simulator
  mkdir -p .niot-test/simulator
  paths=(simulator/auv_gui simulator/auv_scoring simulator/auv_simbridge simulator/auv_worlds)
  if ws_provides_auv_msgs "$(test_ws_root)"; then
    echo "  auv_msgs: using the workspace's own copy"
  else
    echo "  auv_msgs: none in the workspace, exporting the vendored one"
    paths+=(simulator/auv_msgs)
  fi
  git archive "$sha" "${paths[@]}" | tar -x --strip-components=1 -C .niot-test/simulator
  echo "$sha" > "$stamp"
}

# NIOT_WS has to be an absolute path in the environment before Compose reads the
# file: a relative one there would resolve against this repo, not the workspace.
test_compose() {
  NIOT_WS="$(test_ws_root)" exec docker compose "${TEST_FILES[@]}" "$@"
}

# Same reason the competition stack calls the entrypoint by hand: `exec`
# bypasses it, so a non-interactive command would run without ROS on its path.
test_exec() {
  NIOT_WS="$(test_ws_root)" exec docker compose "${TEST_FILES[@]}" exec test /entrypoint.sh "$@"
}

test_running() { docker ps --format '{{.Names}}' | grep -qx niot-test; }

# For the commands that attach to the container rather than start it. `up -d`
# is not a no-op when it is already running: Compose compares the merged config
# it would produce now against the running container, and any environment
# variable that differs between the two invocations — SIM_GUI, SIM_SEED,
# TEST_DOMAIN_ID — counts as a change, so it recreates the container and takes
# the running simulator down with it. `./run.sh test_shell` must not do that.
test_attach_ready() { test_running || test_ensure_up; }

test_ensure_up() {
  local ws
  ws="$(test_ws_root)"
  sync_sim
  # Compose would create these as root if they were missing, leaving a run's
  # artefacts unreadable from the host.
  mkdir -p .niot-test/runs
  grant_x11
  # Both stacks default to ROS_DOMAIN_ID 0 and both start a Gazebo. Running them
  # together is legal but rarely meant: two simulators publish the same topics
  # and the vehicle gets contradictory thrust.
  if docker ps --format '{{.Names}}' | grep -qx niot-sim; then
    echo "WARNING: the competition stack (niot-sim) is up on the same domain." >&2
    echo "  ./run.sh down first, or start this one with TEST_DOMAIN_ID=7." >&2
    echo >&2
  fi
  NIOT_WS="$ws" docker compose "${TEST_FILES[@]}" up -d
}

cmd="${1:-shell}"
shift || true

case "$cmd" in
  build)      exec docker compose "${COMPOSE_FILES[@]}" build "$@" ;;
  up)         ensure_up
              gpu_wanted \
                && echo "Stack is up (NVIDIA runtime active)." \
                || echo "Stack is up (software/iGPU rendering — see ./run.sh gpu)."
              echo "Logs: ./run.sh sim | bridge | controller" ;;
  down)       exec docker compose "${COMPOSE_FILES[@]}" down "$@" ;;

  # The services start their own nodes, so these follow logs rather than
  # launching anything.
  sim)        ensure_up; exec docker compose "${COMPOSE_FILES[@]}" logs -f sim "$@" ;;
  bridge)     ensure_up; exec docker compose "${COMPOSE_FILES[@]}" logs -f domain_bridge "$@" ;;
  controller) ensure_up; exec docker compose "${COMPOSE_FILES[@]}" logs -f competitor "$@" ;;

  shell)      ensure_up; exec docker compose "${COMPOSE_FILES[@]}" exec "${1:-sim}" bash ;;

  # Handy for confirming the domains really are separate: the competitor should
  # see only the bridged topics, with no /scoring/* and no /model/auv/*.
  topics)     ensure_up; in_container "${1:-sim}" ros2 topic list ;;

  # --- the testing stack -------------------------------------------------
  test_build) NIOT_WS="$(test_ws_root)" \
                exec docker compose "${TEST_FILES[@]}" build test "$@" ;;
  test_up)    test_ensure_up
              echo
              echo "Testing container up. Workspace: $(test_ws_root) -> /ros2_ws"
              gpu_wanted \
                && echo "Rendering: NVIDIA runtime active." \
                || echo "Rendering: software/iGPU — see ./run.sh gpu."
              echo
              echo "The first start builds the workspace, which takes a while:"
              echo "  ./run.sh test_logs      watch it, then the simulator"
              echo "  ./run.sh test_shell     a shell beside it, ws sourced"
              echo "  ./run.sh test_ws        rebuild after a dependency change" ;;
  test_down)  test_compose down "$@" ;;
  test_logs|test_log)
              test_compose logs -f test "$@" ;;
  test_shell) test_attach_ready
              NIOT_WS="$(test_ws_root)" \
                exec docker compose "${TEST_FILES[@]}" exec test bash ;;
  # Deliberately not `test_ensure_up`: starting the container *is* a build when
  # the workspace has never been built, and a second colcon writing into the
  # same build base at the same time corrupts both.
  test_ws)    if test_running; then
                test_exec /usr/local/bin/build_ws.sh "$@"
              else
                test_ensure_up
                echo
                echo "Container was not running. Starting it builds the workspace"
                echo "anyway — follow it with ./run.sh test_logs."
              fi ;;
  test_topics) test_attach_ready; test_exec ros2 topic list ;;
  test_sync)  sync_sim --force
              echo "Done. ./run.sh test_ws to rebuild against it." ;;

  # For a package added to the workspace after the image was built. Installs
  # into the running container only — it is gone on the next `test_down`, so
  # fold anything permanent into testing/docker/Dockerfile.
  #
  # -r, because a real team workspace has package.xml entries rosdep cannot
  # resolve (a bare `ament_python` buildtool_depend, a gz-sim8 that is not a
  # rosdep key), and without it one of those aborts the whole run and installs
  # nothing at all.
  test_deps)  test_attach_ready
              test_exec bash -c \
                'rosdep install --ignore-src -y -r --rosdistro jazzy \
                   --from-paths $(/usr/local/bin/build_ws.sh --list-paths)' ;;

  # The Python the team's nodes import but never declare. Same caveat: it lives
  # only until the container is recreated.
  test_pip)   test_attach_ready
              test_exec bash -c \
                'req=/ros2_ws/src/Matsya_ROS2/requirements.txt
                 [[ -f $req ]] || { echo "No $req in this workspace." >&2; exit 1; }
                 pip install --break-system-packages -r "$req"' ;;

  gpu)        echo "NIOT_GPU        = ${NIOT_GPU:-auto}"
              if command -v nvidia-container-runtime >/dev/null 2>&1; then
                echo "toolkit         = installed ($(command -v nvidia-container-runtime))"
              else
                echo "toolkit         = NOT installed"
                echo
                echo "  sudo apt install nvidia-container-toolkit"
                echo "  sudo nvidia-ctk runtime configure --runtime=docker"
                echo "  sudo systemctl restart docker"
              fi
              gpu_wanted \
                && echo "overlay         = docker-compose.gpu.yml (active)" \
                || echo "overlay         = not applied"
              echo "compose files   = ${COMPOSE_FILES[*]}" ;;

  *)          exec docker compose "${COMPOSE_FILES[@]}" "$cmd" "$@" ;;
esac
