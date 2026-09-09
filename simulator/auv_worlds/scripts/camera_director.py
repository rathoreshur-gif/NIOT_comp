#!/usr/bin/env python3
"""Point the Gazebo GUI camera at the vehicle, and keep it there.

Gazebo can follow a model, but it never starts doing so on its own: every
session opened on the static world pose in robosub.sdf with the vehicle a speck
at the far end of the pool, and somebody had to find the right-click menu. At an
outreach stand that is the difference between a game and a puzzle.

WHY THIS IS A SEPARATE PROCESS. /gui/follow and /gui/follow/offset are
advertised by the GUI itself, and gz-transport does not route a request from a
node back to a service advertised by the process that node lives in - the
request is simply never answered. So the obvious implementation, a GUI panel
that calls the services directly, cannot work; it was tried, blocking and
asynchronous, on the Qt thread and on a worker thread, and every variant timed
out while the identical request from a shell answered immediately. This script
is that shell. The FollowCam panel is only a remote control: it publishes a word
on /gui/camera_control, and this process does the work.

Everything goes through the `gz` CLI rather than Python bindings, which the sim
image does not carry - the same reason simulator_bridge shells out for set_pose.

    /gui/camera_control   gz.msgs.StringMsg  in:  follow | free | zoom_in | zoom_out
    /gui/camera_status    gz.msgs.StringMsg  out: "following 2.4" | "free 2.4"
"""

import argparse
import math
import re
import shutil
import subprocess
import sys
import threading
import time

FOLLOW_SERVICE = '/gui/follow'
OFFSET_SERVICE = '/gui/follow/offset'
COMMAND_TOPIC = '/gui/camera_control'
STATUS_TOPIC = '/gui/camera_status'

# The GUI takes its time coming up, and the vehicle is spawned later still, so
# the first attempts are expected to fail and are not worth shouting about.
STARTUP_TIMEOUT = 180.0
RETRY_PERIOD = 2.0
STATUS_PERIOD = 1.0

ZOOM_STEP = 1.25            # one button press, in or out
MIN_DISTANCE = 1.5
MAX_DISTANCE = 30.0


def gz_service(service, reqtype, request, timeout_ms=3000):
    """Call a gz service. True only when Gazebo answered `data: true`."""
    try:
        done = subprocess.run(
            ['gz', 'service', '-s', service,
             '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
             '--timeout', str(timeout_ms), '--req', request],
            capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5.0)
    except (OSError, subprocess.TimeoutExpired):
        return False
    return 'true' in done.stdout


class CameraDirector:

    def __init__(self, target, offset):
        self.target = target
        # Offset in the target's frame. Off-centre on purpose: straight up the
        # stern hides which way the vehicle is turning.
        self.offset = list(offset)
        self.following = False
        self.lock = threading.Lock()

    # -- talking to Gazebo --------------------------------------------------

    def distance(self):
        return math.sqrt(sum(v * v for v in self.offset))

    def send_offset(self):
        x, y, z = self.offset
        return gz_service(OFFSET_SERVICE, 'gz.msgs.Vector3d',
                          f'x: {x:.4f} y: {y:.4f} z: {z:.4f}')

    def set_following(self, following):
        """Follow the target, or release. An EMPTY name is how Gazebo is told
        to stop; there is no separate unfollow service."""
        name = self.target if following else ''
        if not gz_service(FOLLOW_SERVICE, 'gz.msgs.StringMsg', f'data: "{name}"'):
            return False
        with self.lock:
            self.following = following
        if following:
            # The offset only means anything while following, and Gazebo
            # forgets it on release, so it is re-sent on every lock-on.
            self.send_offset()
        return True

    def zoom(self, factor):
        """Scale the offset along its own direction: the camera keeps its angle
        on the vehicle and only changes how far back it sits, which is what zoom
        means for a chase camera."""
        with self.lock:
            distance = self.distance()
            if distance <= 0.0:
                return
            wanted = min(max(distance * factor, MIN_DISTANCE), MAX_DISTANCE)
            scale = wanted / distance
            self.offset = [v * scale for v in self.offset]
        self.send_offset()

    # -- the loops ----------------------------------------------------------

    def acquire(self):
        """Keep asking until the camera locks on, or the timeout runs out."""
        deadline = time.monotonic() + STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            if self.set_following(True):
                print(f'[camera_director] following {self.target} at '
                      f'{self.distance():.1f} m', flush=True)
                return True
            time.sleep(RETRY_PERIOD)
        print(f'[camera_director] gave up waiting for {FOLLOW_SERVICE} and '
              f'"{self.target}" after {STARTUP_TIMEOUT:.0f}s; the camera stays '
              'where the world file put it.', file=sys.stderr, flush=True)
        return False

    def publish_status(self):
        """Republish on a timer, not only on change: gz transport does not
        latch, so a panel that connects later would show nothing."""
        while True:
            with self.lock:
                state = 'following' if self.following else 'free'
                text = f'{state} {self.distance():.1f}'
            subprocess.run(
                ['gz', 'topic', '-t', STATUS_TOPIC, '-m', 'gz.msgs.StringMsg',
                 '-p', f'data: "{text}"'],
                capture_output=True, text=True)
            time.sleep(STATUS_PERIOD)

    def listen(self):
        """Read commands off the panel's topic, one per line of `gz topic -e`."""
        process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', COMMAND_TOPIC],
            stdout=subprocess.PIPE, text=True)
        for line in process.stdout:
            match = re.search(r'data:\s*"([^"]*)"', line)
            if not match:
                continue
            command = match.group(1).strip()
            if command == 'follow':
                self.set_following(True)
            elif command == 'free':
                self.set_following(False)
            elif command == 'zoom_in':
                self.zoom(1.0 / ZOOM_STEP)
            elif command == 'zoom_out':
                self.zoom(ZOOM_STEP)
            else:
                continue
            print(f'[camera_director] {command} -> '
                  f'{"following" if self.following else "free"} at '
                  f'{self.distance():.1f} m', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', default='auv')
    parser.add_argument('--offset', default='-2.0,1.0,1.0',
                        help='x,y,z in the target frame')
    args = parser.parse_args()

    if shutil.which('gz') is None:
        print('[camera_director] no `gz` on PATH; not driving the camera.',
              file=sys.stderr)
        return 0

    try:
        offset = [float(v) for v in args.offset.split(',')]
        if len(offset) != 3:
            raise ValueError
    except ValueError:
        print(f'[camera_director] --offset must be x,y,z; got {args.offset!r}',
              file=sys.stderr)
        return 2

    director = CameraDirector(args.target, offset)
    threading.Thread(target=director.publish_status, daemon=True).start()
    director.acquire()
    # Carries on serving the buttons even if the initial lock-on failed: the
    # vehicle may be spawned late, and "Third person" then does the job by hand.
    director.listen()
    return 0


if __name__ == '__main__':
    sys.exit(main())
