#!/usr/bin/env bash
# Thin wrapper around docker compose that deals with the X11 handshake, which
# is easy to forget and fails with an opaque Qt error when missed.
#
#   ./run.sh build         build (or rebuild) both images
#   ./run.sh up            start sim + domain_bridge + competitor
#   ./run.sh sim           follow the simulator's logs
#   ./run.sh bridge        follow the domain bridge's logs
#   ./run.sh controller    follow the competitor controller's logs
#   ./run.sh shell [svc]   open a shell in a service (default: sim)
#   ./run.sh topics [svc]  list the topics that service can see
#   ./run.sh down          stop and remove everything
#
# Anything else is passed through to `docker compose` verbatim.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

# Gazebo's GUI is an X client; without this the container's connection to the
# host display is refused. Harmless to re-run, and scoped to local connections.
grant_x11() {
  command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
}

# `docker compose exec` bypasses the ENTRYPOINT, so a non-interactive command
# would run without ROS on its path. Calling the entrypoint explicitly restores
# it. Interactive shells get the same treatment via ~/.bashrc.
in_container() { exec docker compose exec "$1" /entrypoint.sh "${@:2}"; }

ensure_up() { grant_x11; docker compose up -d; }

cmd="${1:-shell}"
shift || true

case "$cmd" in
  build)      exec docker compose build "$@" ;;
  up)         ensure_up
              echo "Stack is up. Logs: ./run.sh sim | bridge | controller" ;;
  down)       exec docker compose down "$@" ;;

  # The services now start their own nodes, so these follow logs rather than
  # launching anything.
  sim)        ensure_up; exec docker compose logs -f sim "$@" ;;
  bridge)     ensure_up; exec docker compose logs -f domain_bridge "$@" ;;
  controller) ensure_up; exec docker compose logs -f competitor "$@" ;;

  shell)      ensure_up; exec docker compose exec "${1:-sim}" bash ;;

  # Handy for confirming the domains really are separate: the competitor should
  # see only the bridged topics, with no /scoring/* and no /model/auv/*.
  topics)     ensure_up; in_container "${1:-sim}" ros2 topic list ;;

  *)          exec docker compose "$cmd" "$@" ;;
esac
