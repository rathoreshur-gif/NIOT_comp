# NIOT AUV Competition

Simulator and competitor template for the NIOT autonomous underwater vehicle
competition. Everything runs in Docker: clone this repo, build once, and you have
a Gazebo pool, a scored course and a working reference controller to replace.

> **Status: draft.** The competition rules, scoring weights and run lifecycle are
> not final. This README will be updated as they are settled.

```
sim              ROS_DOMAIN_ID=0    Gazebo, scoring, simulator_bridge
  |
domain_bridge                       whitelist only
  |
competitor       ROS_DOMAIN_ID=1    your code
```

The simulator and your controller run on **separate ROS domains**. The only path
between them is a topic whitelist, so your code cannot see the scoring topics or
the raw Gazebo state — only what a real vehicle's sensors would give you.

## Layout

```
NIOT_comp/
├── docker-compose.yml       # the whole stack
├── run.sh                   # thin wrapper around docker compose
├── simulator/               # the competition simulator — you do not need to edit this
│   ├── auv_msgs/            # message definitions (shared with your container)
│   ├── auv_gui/             # Gazebo GUI panels
│   ├── auv_scoring/         # task flaggers and the score keeper
│   ├── auv_simbridge/       # Gazebo <-> ROS bridge, sensor publishing
│   ├── auv_worlds/          # world generation, course layout, launch files
│   └── docker/              # simulator image, domain bridge whitelist, docs
└── competitor/              # YOUR CODE GOES HERE
    ├── competitor_controller/
    └── docker/
```

## Requirements

- Linux with X11 (for the Gazebo GUI; headless works anywhere)
- Docker Engine and Docker Compose v2
- ~25 GB disk for the images, 8 GB RAM
- NVIDIA GPU optional but strongly recommended — see below

### GPU

Gazebo renders the camera sensors on the GPU, so an NVIDIA card is worth a large
share of the real-time factor. `run.sh` detects the NVIDIA Container Toolkit and
enables the GPU automatically when it is present:

```bash
./run.sh gpu        # report what is in use, and how to fix it if not
```

If it reports the toolkit is missing, install it once:

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Without it the container falls back to software or integrated-GPU rendering and
still works, just slower. Override the detection with `NIOT_GPU=0` or
`NIOT_GPU=1` if you need to.

## Quick start

```bash
git clone <this-repo> NIOT_comp
cd NIOT_comp

./run.sh build      # 15-20 minutes the first time. Both images.
./run.sh up         # starts simulator + bridge + your controller
./run.sh controller # follow your controller's logs
./run.sh down       # stop everything
```

`up` runs detached, so nothing attaches to your terminal. Follow logs with
`./run.sh sim`, `./run.sh bridge` or `./run.sh controller` — Ctrl+C there only
stops the log follower, not the stack. Use `./run.sh down` to actually stop.

Useful knobs:

```bash
SIM_GUI=false ./run.sh up       # headless, no X11 needed
SIM_SEED=1234 ./run.sh up       # replay a specific course
SIM_LIFECYCLE=false ./run.sh up # no run manager: thrusters live from boot, as before
```

**Expect a real-time factor around 0.5** on a mid-range laptop. The camera
streams are the cost. Use headless mode for faster iteration.

## What your code can see and send

Your container is on `ROS_DOMAIN_ID=1`, so `ros2 topic list` shows only these.

**Inputs:**

| Topic | Type |
|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/localization/pose` | `auv_msgs/msg/AuvState` |
| `/front_camera/image_raw` | `sensor_msgs/msg/Image` |
| `/bottom_camera/image_raw` | `sensor_msgs/msg/Image` |
| `/front_camera/rgbd_frame` | `auv_msgs/msg/RGBDFrame` (RGB + depth) |
| `/simulator/run_state` | `auv_msgs/msg/RunState` (see [The run](#the-run)) |

**What you can send back:**

| Topic | Type | |
|---|---|---|
| `/controller/thruster_forces` | `auv_msgs/msg/ThrusterForces` | the only thing that actuates the vehicle |
| `/simulator/run_control` | `std_msgs/msg/String` | `start`, `end` or `reset` |

Publishing anything else has no effect on the simulation. The whitelist is
`simulator/docker/domain_bridge.yaml`.

## The run

A run is armed, not free-running. **The thrusters are dead until the run
starts**, so publishing thrust before that moves nothing. That is deliberate:
otherwise the clock effectively starts when Gazebo does and a controller that
takes fifteen seconds to boot is penalised for its own startup.

From your container, one word on a topic:

```bash
ros2 topic pub --once /simulator/run_control std_msgs/msg/String "data: start"
ros2 topic pub --once /simulator/run_control std_msgs/msg/String "data: end"
ros2 topic pub --once /simulator/run_control std_msgs/msg/String "data: reset"
ros2 topic echo /simulator/run_state
```

`domain_bridge` carries topics and not services, which is why these are strings
on a topic rather than service calls. In the simulator container the same three
are `std_srvs/srv/Trigger` services — `/simulator/start_run`,
`/simulator/end_run`, `/simulator/reset_run` — and they report back in their
response, so that is the side to drive them from when you are debugging.

`/simulator/run_state` publishes at 5 Hz throughout:

| Field | |
|---|---|
| `state`, `state_label` | `IDLE` → `RUNNING` → `FINISHED` |
| `elapsed`, `remaining`, `time_limit` | seconds of simulated time |
| `score` | your live total, time bonus included once it is paid |
| `gate_complete`, `slalom_complete` | whether you have run each task at all |
| `run_index`, `run_id` | which run and which course layout |

### Ending early is worth points

The time you do not use pays **100 points per minute**, out of a **20 minute**
limit. Both numbers live under `run:` in
`simulator/auv_worlds/config/competition_config.yaml`, which is bind-mounted
into the simulator, so changing the limit between runs is an edit and a restart
rather than a rebuild:

```bash
$EDITOR simulator/auv_worlds/config/competition_config.yaml   # run.time_limit
docker compose restart sim
```

The bonus is only paid if the vehicle has actually **run the gate and the
slalom** — through the gate, and through all three slalom layers. Whether those
passes scored does not matter: a slalom layer taken on a non-scoring side still
counts as having attempted the task. Without that condition, starting a run and
immediately ending it would be the highest-scoring strategy in the competition.

Every task pays **once per thing done**: the gate once (either half, 50), each
slalom layer once (100), each bin once (500), each torpedo hole once (500) and
each octagon object once (1000). The marker and the torpedo reload on a button
press, so scoring per drop or per shot would make a single bin an unlimited
supply of points.

Reaching the time limit ends the run exactly as `end` does, except that there is
no time left to pay for. Either way the scorecard is written to
`.docker-runs/result_<run_id>_run<n>.json`.

### Practising

`reset` re-poses the vehicle and re-seeds the course **without restarting
Gazebo** — a couple of seconds instead of a 15-20 s boot, which is the
difference between forty attempts in an evening and ten. It clears the score and
leaves you in `IDLE`, ready for another `start`.

Two things it cannot undo, both physical: a marker that has already been dropped
stays on the pool floor, and a grasped pickup stays grasped — those are welds
that only a relaunch remakes, so a reset run cannot score the bins again. And
while the run is `IDLE` the vehicle is unpowered, so it drifts up toward the
surface; `start` promptly after a `reset` if you want to begin from the spawn
depth.

## Units — read this before writing any control code

None of these are what you would guess, and each one produces a controller that
looks plausible and quietly diverges:

- **Positions are in centimetres**, not metres.
- **+z is DOWN.** A larger z means deeper; commanding a negative z asks the
  vehicle to rise.
- **Orientation is in degrees**, not radians.
- `velocity` is cm/s and already body-frame. `angular_velocity` is rad/s and is
  **not** scaled.
- Thruster forces are newtons, saturating at **±40 N**.

## Running your own code

`competitor/competitor_controller/competitor_controller/basic_controller.py` is a
minimal reference: PID on pose error, a fixed mixer to the eight thrusters,
saturation cap. It controls surge, sway, heave and yaw; roll and pitch are left
to the vehicle's own righting moment. Replace it.

Your code is baked into the image at build time, so by default a change means:

```bash
docker compose build competitor && ./run.sh up
```

For a faster loop, create `docker-compose.override.yml` (Compose picks it up
automatically) so the container idles and your source is live:

```yaml
services:
  competitor:
    command: sleep infinity
    volumes:
      - ./competitor/competitor_controller:/ros2_ws/src/competitor_controller
```

Then:

```bash
./run.sh up
docker compose exec competitor bash
cd /ros2_ws && colcon build --symlink-install --packages-select competitor_controller
source install/setup.bash
ros2 launch competitor_controller competitor.launch.py
```

With `--symlink-install` your Python edits take effect on a node restart — no
rebuild unless you change `setup.py` or `package.xml`. **Rebuild cleanly without
the override before submitting.**

⚠️ Only one node may publish `/controller/thruster_forces`. If you launch your
own controller while the container's built-in one is running, they will fight and
the vehicle will do something incoherent. That is what `command: sleep infinity`
above prevents.

See `competitor/README.md` for more detail.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Opaque Qt / X11 error on startup | `xhost +local:root`, or use `SIM_GUI=false`. `run.sh` normally does this for you. |
| No camera frames in the competitor | Check the domain bridge is up: `./run.sh bridge` |
| Vehicle rises when told to descend | Flip `heave_sign:=-1.0` on the controller launch |
| Low real-time factor | Expected — see `simulator/docker/PERFORMANCE.md` |
| `container_name` already in use | Another copy of the stack is running: `./run.sh down` |

## License

Apache License 2.0 — see [LICENSE](LICENSE).
