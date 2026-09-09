#!/usr/bin/env bash
# Gamepad demo mode: the competition simulator, driven by hand from a pad.
#
#   ./demo.sh up            start the stack with the gamepad controller
#   ./demo.sh log           follow the gamepad node's output
#   ./demo.sh status        watch what the controller thinks the pad is doing
#   ./demo.sh reset         re-pose the vehicle and re-seed the course
#   ./demo.sh check         is the pad visible, and where
#   ./demo.sh down          stop everything
#
# Anything else is passed through to `docker compose` with the demo overlay
# applied, so `./demo.sh build competitor` and friends work.
#
# This is run.sh with one extra compose file. The simulator, the scoring and the
# domain bridge are all untouched; only the competitor container differs, and it
# still drives the vehicle through the same /controller/thruster_forces topic a
# competitor is limited to.
#
#   JOY_DEV=/dev/input/js1 ./demo.sh up     # pad is not js0
#   DEMO_SPEED=0.30 ./demo.sh up            # slower, for a crowded table
#   DEMO_HEAVE_SIGN=-1.0 ./demo.sh up       # if it rises when told to dive
#   SIM_LIFECYCLE=false ./demo.sh up        # thrusters live from boot, no START
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

grant_x11() {
  command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
}

# Same detection run.sh uses: test for the binary, not `docker info`, because
# Docker keeps a stale `nvidia` runtime registration on machines where the
# toolkit was never installed.
gpu_wanted() {
  case "${NIOT_GPU:-auto}" in
    0|false|off|no) return 1 ;;
    1|true|on|yes)  return 0 ;;
    *) command -v nvidia-container-runtime >/dev/null 2>&1 ;;
  esac
}

JOY_DEV="${JOY_DEV:-/dev/input/js0}"
export JOY_DEV

COMPOSE_FILES=(-f docker-compose.yml)
gpu_wanted && COMPOSE_FILES+=(-f docker-compose.gpu.yml)
COMPOSE_FILES+=(-f docker-compose.demo.yml)
# The user's own override still wins, as it does in run.sh: last file listed.
[[ -f docker-compose.override.yml ]] && COMPOSE_FILES+=(-f docker-compose.override.yml)

# The stack starts fine without a pad - the node waits and reconnects by itself,
# and /dev/input is bind-mounted so a pad plugged in later just appears. So this
# warns rather than refusing: at a stand, "start the sim now, pad in a minute"
# is a normal thing to want.
check_pad() {
  [[ -e "$JOY_DEV" ]] && return 0
  echo "WARNING: no joystick at $JOY_DEV." >&2
  if compgen -G "/dev/input/js*" >/dev/null; then
    echo "  Pads that are present: $(ls -1 /dev/input/js* | tr '\n' ' ')" >&2
    echo "  The node takes any js* it finds, so this will most likely still" >&2
    echo "  work. To pin one: JOY_DEV=/dev/input/jsN ./demo.sh up" >&2
  else
    echo "  No /dev/input/js* at all. Plug the pad in - it will be picked up" >&2
    echo "  without restarting anything. If it stays invisible and it is a" >&2
    echo "  Zebronics or similar, switch it to XInput mode (a switch on the" >&2
    echo "  back, or holding HOME), then check with ./demo.sh check" >&2
  fi
  echo >&2
}

cmd="${1:-up}"
shift || true

case "$cmd" in
  up)
      check_pad
      grant_x11
      docker compose "${COMPOSE_FILES[@]}" up -d
      echo
      echo "Demo stack up. Pad $JOY_DEV -> container /dev/input/js0."
      gpu_wanted || echo "(software/iGPU rendering - see ./run.sh gpu)"
      cat <<'CONTROLS'

  Left stick     move, in the VEHICLE's frame - forward is wherever the nose points
  Right stick    X aims the nose, Y rises and dives
  RT / LT        boost / precision, both analogue
  A              drop marker      Y  fire torpedo     X  gripper open/close
  LB             anchor here      D-pad up/down  one gear;  D-pad left  NORMAL
  START          start the run    BACK  end the run

  L1+L2+R1+R2    squeeze all four together for the next gear:
                 SLOW - NORMAL - FAST - TURBO - SLOW ...
                 The gear shows on the scoreboard in the Gazebo window.

  One marker, one torpedo. A and Y toggle: press to drop or fire, press
  again and the payload is put back on the hull, ready for another go.

  Reset is not on the pad - it re-seeds the whole course, so it belongs to
  whoever is running the stand:  ./demo.sh reset

  Let go of the sticks and it freezes - position, heading and depth.

  Logs:  ./demo.sh log        Live state:  ./demo.sh status
CONTROLS
      ;;

  down)   exec docker compose "${COMPOSE_FILES[@]}" down "$@" ;;

  log|logs)
      exec docker compose "${COMPOSE_FILES[@]}" logs -f competitor "$@" ;;

  sim)    exec docker compose "${COMPOSE_FILES[@]}" logs -f sim "$@" ;;

  # /gamepad/status is published inside the competitor's domain only - it is not
  # on the bridge whitelist - so it has to be echoed from that container.
  status)
      exec docker compose "${COMPOSE_FILES[@]}" exec competitor \
        /entrypoint.sh ros2 topic echo /gamepad/status ;;

  # Between competitors: re-pose the vehicle, re-seed the course, clear the
  # score. Deliberately off the pad - a thumb should not be able to wipe a run.
  # The service lives on the simulator's domain, so it is called from there.
  reset)
      exec docker compose "${COMPOSE_FILES[@]}" exec sim \
        /entrypoint.sh ros2 service call /simulator/reset_run std_srvs/srv/Trigger ;;

  shell)  exec docker compose "${COMPOSE_FILES[@]}" exec "${1:-competitor}" bash ;;

  # Everything the pad touches, host side, before Docker is involved at all.
  check)
      echo "JOY_DEV = $JOY_DEV"
      if [[ -e "$JOY_DEV" ]]; then
        echo "device  = present"
        ls -l "$JOY_DEV"
      else
        echo "device  = MISSING"
      fi
      echo
      echo "All pads seen by the kernel:"
      ls -1 /dev/input/js* 2>/dev/null || echo "  (none)"
      echo
      grep -i -B1 -A5 -E "pad|joystick|gamepad" /proc/bus/input/devices 2>/dev/null \
        | grep -E "^N:|^H:" || true
      ;;

  *)      exec docker compose "${COMPOSE_FILES[@]}" "$cmd" "$@" ;;
esac
