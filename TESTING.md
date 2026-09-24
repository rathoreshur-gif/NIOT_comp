# The testing container

A second, separate stack for the team: one container with the whole workspace
in it — the Matsya ROS 2 code and the simulator — running the simulator and the
simbridge, for ordinary development rather than for a competition run.

```bash
./run.sh test_build     # once per machine (and after a dependency change)
./run.sh test_up        # simulator + simbridge, on your own workspace
./run.sh test_shell     # a shell beside them, workspace sourced
./run.sh test_down
```

The first `test_up` builds the workspace inside the container, which takes a
while; `./run.sh test_logs` shows it happening. Every start after that reuses
the build.

## What is in it

The image contains **no source code at all** — only ROS Jazzy, Gazebo, Qt and
the Python the team's nodes import. The workspace arrives at runtime as a bind
mount:

| Container path | Host path |
| --- | --- |
| `/ros2_ws` | your own workspace (`NIOT_WS`, default: this repo's grandparent) |
| `/ros2_ws/src/niot_simulator` | `.niot-test/simulator`, exported from a pinned commit |
| `/opt/testws` | a Docker named volume: `build/`, `install/`, `log/` |
| `/tmp/matsya_competition` | `.niot-test/runs`, generated worlds and ground truth |

So it is *your* checkout that builds and runs in there — your branches, your
half-finished node, your edits, live. Nothing is copied in, and nothing of
yours is baked into an image.

Two consequences worth knowing:

- **Python edits need no rebuild.** The workspace is built with
  `--symlink-install`, so editing a node on the host takes effect on the next
  `ros2 run`. C++ and message changes still need `./run.sh test_ws`. The one
  mark this leaves on your checkout is an `*.egg-info` directory beside each
  Python package, which is what a symlink install is; `TEST_SYMLINK=false`
  turns it off at the cost of rebuilding for every edit.
- **The container's build never touches your host `build/` and `install/`.**
  They hold your native colcon build; a container writing into the same trees
  would leave one workspace holding two incompatible sets of artefacts. The
  container builds into a named volume instead, which survives `test_down` and
  is invisible to the host.

## Why the simulator is pinned, and to what

`/ros2_ws/src/niot_simulator` is exported from commit `56164bd`, the
competition release as it stood **before the demo build landed** — no gamepad
path, no START/END run lifecycle, no payload/torpedo models, no FollowCam, no
run manager. Those exist for showing the simulator at a stand, and they get in
the way of testing a controller: the lifecycle gate in particular keeps the
thrusters dead until something sends START.

`./run.sh test_up` refreshes the export when the pin moves and otherwise leaves
it alone, so it does not force a rebuild on every start. To point it somewhere
else:

```bash
NIOT_TEST_REF=main ./run.sh test_sync
./run.sh test_ws                        # rebuild against it
```

The working tree of `simulator/` is **not** what this container runs. That is
the competition/demo stack, and it stays with `./run.sh up` and `./demo.sh up`.

## Which packages get built

`build_ws.sh` discovers packages in the mounted workspace itself, because
colcon will not build a workspace containing two packages of the same name — it
stops rather than choosing. Two collisions are resolved for you:

- `Matsya_ROS2_Simulator` is skipped entirely. It is the team's own copy of
  `auv_worlds` / `auv_gui` / `auv_scoring` / `auv_simbridge`, and the pinned
  tree is the one this container promises to run. `TEST_SKIP_DIRS` changes
  this.
- `auv_msgs` comes from your workspace (`Matsya_ROS2/auv_msgs`), because that
  is what the team's nodes are written against. The copy vendored in this repo
  is exported only for a workspace that has none.

`NIOT_comp` itself is skipped: its tree carries a `COLCON_IGNORE`.

The build runs with `--continue-on-error` on purpose — a teammate's
half-written package should cost them that package, not the simulator they were
trying to test against. If `auv_msgs`, `auv_worlds` or `auv_simbridge` fails,
that *is* fatal and is reported as such.

```bash
./run.sh test_ws                  # rebuild everything
./run.sh test_ws auv_controller   # just this package and its dependencies
```

## Talking to it

`network_mode: host` and `ipc: host`, on `ROS_DOMAIN_ID=0`, so the container's
ROS graph is the host's ROS graph. A `ros2 topic echo` in a plain host terminal
sees the simulator, and a node started on the host talks to it — no extra
setup, as long as the host has Jazzy and the message definitions.

What the simbridge puts on the graph is the vehicle's real interface:

| Direction | Topics |
| --- | --- |
| commands in | `/controller/thruster_forces`, `/controller/global_forces` |
| state out | `/localization/pose`, `/model/auv/odometry`, `/model/auv/imu_*`, `/model/auv/pressure` |
| cameras | `/front_camera/image_raw`, `/front_camera/rgbd_frame`, `/bottom_camera/image_raw` |
| services | `/simulator/kill_thrusters`, `/simulator/open_gripper`, `/simulator/close_gripper`, `/simulator/drop_marker_*` |

Nothing is filtered. Unlike the competition stack there is no domain bridge and
no whitelist, so ground truth and `/scoring/*` are visible if you turn scoring
on — which is the point of a testing container and exactly why it is not the
one used for a run.

## Knobs

Environment variables, all read at `test_up`:

| Variable | Default | Meaning |
| --- | --- | --- |
| `NIOT_WS` | this repo's grandparent | which workspace to mount |
| `SIM_GUI` | `true` | `false` runs Gazebo headless — no X11 needed |
| `SIM_SEED` | `auto` | an integer replays a specific course |
| `SIM_SCORING` | `false` | `true` runs the task flaggers |
| `SIM_TELEOP` | `false` | `true` runs the keyboard teleop — see below |
| `SIM_PAUSED` | `false` | `true` starts Gazebo paused |
| `TEST_DOMAIN_ID` | `0` | move off the default ROS domain |
| `TEST_SKIP_DIRS` | `Matsya_ROS2_Simulator` | source dirs to leave out of the build |
| `TEST_SYMLINK` | `true` | `false` for a plain copy install |
| `TEST_REBUILD` | `0` | `1` rebuilds the workspace on start |
| `NIOT_GPU` | `auto` | `1`/`0` to force the NVIDIA overlay on or off |
| `NIOT_TEST_REF` | `56164bd` | which commit the simulator is exported from |

`SIM_TELEOP=true` only when driving by hand with no controller running: the
teleop node publishes to the same `/model/auv/joint/*/cmd_thrust` topics as the
simbridge, at 50 Hz, and freezes position, heading and depth whenever no key is
held — so left on it silently station-keeps against whatever you are testing.

## What the image installs for Matsya_ROS2

The image has every third-party module a Matsya_ROS2 node imports, so each
`ros2 run` / `ros2 launch` in that repo starts:

| Package | Needs |
| --- | --- |
| `auv_controller` | `simple-pid`, pandas, matplotlib |
| `auv_navigator` | `ruckig` |
| `auv_vision` | `ultralytics` + torch (CPU build), `depthai` 3.x, cv_bridge |
| `auv_drivers` | pyzmq, pyserial, `depthai`, tkinter |
| `auv_acoustics` | `nidaqmx`, scipy, matplotlib |
| `auv_map` | pyqtgraph + PyQt5, flask, pandas |
| `auv_localization`, `auv_mission_control` | numpy, scipy |

The last step of the image build imports all of these. If any is missing,
`./run.sh test_build` fails right there, not later in someone's `ros2 run`.

The hardware SDKs install and import fine, but they only work with the real
device: a DAQ needs NI's driver on the host, and an OAK camera needs USB
passthrough. Their nodes start in the container, then fail when they look for
the hardware. In the simulator, the camera, IMU and pressure data come from
the simbridge instead.

The vision stack uses the **CPU** torch wheel, a few hundred MB instead of about
3 GB for the CUDA build. Two ways to change that:

```bash
./run.sh test_build --build-arg TEST_VISION_DEPS=false   # leave it out
./run.sh test_build --build-arg TORCH_INDEX=https://download.pytorch.org/whl/cu124
```

apt's numpy (1.26) is pinned for every pip install through `PIP_CONSTRAINT`,
including one typed in `test_shell`. rclpy, cv_bridge and the apt
scipy/opencv are all built against it, and a wheel that pulled in numpy 2
would break them.

## When something is missing

The image carries a fixed dependency set. A package added to the workspace
later may want something it does not have:

```bash
./run.sh test_deps    # rosdep the mounted workspace
./run.sh test_pip     # pip install Matsya_ROS2/requirements.txt
```

`test_deps` runs with `-r`, so keys it cannot resolve are reported and skipped.
Examples are a bare `ament_python` buildtool_depend, or a `gz-sim8` that is not
a rosdep key. Without `-r`, one of those would abort the run and install
nothing. `test_pip` will stop on a hardware SDK that has no wheel for this
Python. If that happens, install the rest by hand.

Both install into the *running* container only, so they are gone after
`test_down`. Anything permanent belongs in `testing/docker/Dockerfile`: add the
package, add it to the import check at the end of that file, then run
`./run.sh test_build`.

## Both stacks at once

Don't, without moving one off domain 0. The competition stack (`./run.sh up`)
and this one both start a Gazebo and both default to `ROS_DOMAIN_ID=0`: two
simulators publishing the same topics, and a vehicle taking thrust commands
from both. `test_up` warns when it spots `niot-sim` running. If you genuinely
want both:

```bash
./run.sh down            # or
TEST_DOMAIN_ID=7 ./run.sh test_up
```

They are otherwise fully independent — separate images, separate containers,
separate Compose projects — so `./run.sh down` cannot take the testing
container with it, and `./run.sh test_down` cannot take down a competition run.
