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
- GPU optional — see `simulator/docker/PERFORMANCE.md`

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
SIM_GUI=false ./run.sh up      # headless, no X11 needed
SIM_SEED=1234 ./run.sh up      # replay a specific course
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
| `/front_camera/oakd_frame` | `auv_msgs/msg/StereoVisionFrame` |
| `/front_camera/rgbd_frame` | `auv_msgs/msg/RGBDFrame` |

**The one output that reaches the vehicle:**

| Topic | Type |
|---|---|
| `/controller/thruster_forces` | `auv_msgs/msg/ThrusterForces` |

Publishing anything else has no effect on the simulation. The whitelist is
`simulator/docker/domain_bridge.yaml`.

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
