# Running the competition stack in Docker

Three containers. The simulator starts itself, and the competitor's code runs on
a separate ROS domain that reaches it only through a whitelist.

```
sim              ROS_DOMAIN_ID=0    competition.launch.py + simulator_bridge
  |
domain_bridge                       joins domains 0 and 1, whitelist only
  |
competitor       ROS_DOMAIN_ID=1    basic_controller
```

`sim` is ROS 2 Jazzy + Gazebo Harmonic with `auv_gui`, `auv_scoring`,
`auv_simbridge`, `auv_worlds` and `auv_msgs` prebuilt into `/ros2_ws/install`.
`competitor` is a much smaller ros-base image with only `auv_msgs` and the
controller — no Gazebo, no scoring code.

## Layout requirement

The image needs the sibling `Matsya_ROS2` checkout, because `auv_simbridge`
imports message types from `auv_msgs`:

```
ros2_ws/src/
├── Matsya_ROS2/auv_msgs/          <- required
├── Matsya_ROS2_Simulator/         <- you are here
└── competitor/                    <- the competitor's container
```

Only `auv_msgs` is copied in; the rest of the competitor stack is excluded. This
is why `docker-compose.yml` sets `context: ..`.

## Quick start

```bash
cd Matsya_ROS2_Simulator

./docker/run.sh build      # ~15-20 min the first time, both images
./docker/run.sh up         # starts all three
./docker/run.sh sim        # follow the simulator's logs
```

Nothing else is needed — the sim container runs `competition.launch.py` with
`simulator_bridge:=true` as its main process, and the competitor container starts
its controller. To drive it manually instead:

```bash
docker compose up -d
docker compose logs -f sim
docker compose exec sim bash          # ROS is sourced via ~/.bashrc
docker compose down
```

### Launch knobs

Read from the environment, so you do not have to edit `docker-compose.yml`:

```bash
SIM_GUI=false     docker compose up   # headless, no X11 needed
SIM_SEED=1234     docker compose up   # replay a specific course
SIM_SCORING=false docker compose up   # skip the task flaggers
SIM_TELEOP=true   docker compose up   # see the warning below first
```

Anything else `competition.launch.py` accepts (`paused`, `current`, `config`,
`output_dir`) can be added to the `command:` block.

### Do not run teleop alongside a controller

`teleop` defaults to **on** in `competition.launch.py`, and the compose command
forces it **off**. That is not cosmetic. The teleop node publishes to the same
`/model/auv/joint/*/cmd_thrust` topics as `simulator_bridge`, at 50 Hz, and by
design freezes position, heading and depth whenever no key is held.

Left enabled, it station-keeps against the competitor's controller. The failure
is quiet and easy to misread: the vehicle sits still while the controller
saturates its thrusters, and the joint commands reaching Gazebo are an interleaved
mixture of both publishers, so they match neither. Turn `SIM_TELEOP=true` on only
when driving by hand with no controller running.

## The domain bridge

`docker/domain_bridge.yaml` is the contract between the two sides, and doubles
as the whitelist — a topic that is not listed there does not reach the
competitor:

| Direction | Topic | Type |
|---|---|---|
| 0 → 1 | `/clock` | `rosgraph_msgs/msg/Clock` |
| 0 → 1 | `/localization/pose` | `auv_msgs/msg/AuvState` |
| 0 → 1 | `/front_camera/image_raw` | `sensor_msgs/msg/Image` |
| 0 → 1 | `/bottom_camera/image_raw` | `sensor_msgs/msg/Image` |
| 0 → 1 | `/front_camera/rgbd_frame` | `auv_msgs/msg/RGBDFrame` |
| 0 → 1 | `/simulator/run_state` | `auv_msgs/msg/RunState` |
| 1 → 0 | `/controller/thruster_forces` | `auv_msgs/msg/ThrusterForces` |
| 1 → 0 | `/simulator/run_control` | `std_msgs/msg/String` |

`/scoring/*`, the ground-truth flaggers, the raw `/model/auv/*` topics and every
`/simulator/*` **service** stay inside domain 0. The run lifecycle is the one
thing that has to reach the competitor - it needs to know it is live, and to be
able to end its own run - so `run_manager` mirrors its three Trigger services
onto the last two topics above: state out, one-word commands back. Confirm the
rest still holds:

```bash
./docker/run.sh topics sim          # the full graph
./docker/run.sh topics competitor   # only the seven above
```

The file is bind-mounted, so editing the whitelist needs only
`docker compose restart domain_bridge` — no rebuild.

The `domain_bridge` service reuses `matsya-sim:jazzy` rather than building a
third image: bridging `auv_msgs` topics needs `auv_msgs` on the
`AMENT_PREFIX_PATH`, and that image already has it.

## Networking

All three services use host networking **and** `ipc: host`. Host networking
alone is not enough: FastDDS advertises shared-memory locators, and a service in
a different IPC namespace cannot reach them, so discovery stalls rather than
falling back cleanly to UDP.

Isolation comes from the domain IDs, not the network — which is also why a
controller run directly on the host sees the simulator if you set
`ROS_DOMAIN_ID=0`, and sees the competitor's side of the bridge at
`ROS_DOMAIN_ID=1`. That is useful for debugging.

## Graphics

The default uses the host's DRI render nodes (`/dev/dri`), which covers Intel
and AMD, and NVIDIA when PRIME offloading is not required.

**NVIDIA.** The toolkit is *not* installed on this machine, so the container
currently falls back to the Intel iGPU even though the RTX 4050 is visible to
it. To enable it:

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

then add to the `sim` service in `docker-compose.yml`:

```yaml
    runtime: nvidia
```

The `NVIDIA_VISIBLE_DEVICES` / `NVIDIA_DRIVER_CAPABILITIES` variables are
already set and are ignored under the default runtime. Adding `runtime: nvidia`
*before* installing the toolkit fails with `"nvidia-container-runtime":
executable file not found`.

See [PERFORMANCE.md](PERFORMANCE.md) — the GPU is not the main RTF bottleneck.

**No GPU / remote machine.** Drop the `devices:` block and add
`- LIBGL_ALWAYS_SOFTWARE=1` to `environment:`. Gazebo will fall back to
software rendering, which is slow but functional. For an unattended run prefer
`SIM_GUI=false`, which needs no display at all.

## Performance

If RTF is low, it is almost certainly not Docker: headless, container and host
measure the same (~0.40 vs ~0.42). `simulator_bridge` roughly halves RTF on both
because of the camera streams, and bridging those cameras across domains adds to
that. See [PERFORMANCE.md](PERFORMANCE.md) for measurements and a tuning recipe
that reaches a stable 1.0 with ~31% headroom.

If image latency across the bridge becomes the bottleneck, the knob is
`reliability: best_effort` on the camera entries in `domain_bridge.yaml` — but
that then *requires* competitors to subscribe best-effort, which is why it is not
the default. Dropping a camera topic from the whitelist entirely is cheaper still.

## Notes

- `init: true` on the sim service matters: `ros2 launch` is PID 1 there and would
  otherwise reap no zombies and handle signals poorly. `stop_grace_period: 30s`
  gives Gazebo time to shut down instead of being SIGKILLed at the 10s default.
- `SYS_NICE` is granted because `competition.launch.py` starts Gazebo under
  `nice -n -10`; without the capability the renice silently fails.
- Generated worlds and ground-truth JSON are bind-mounted to `./.docker-runs` so
  a run's artefacts stay readable from the host.
- Source is copied, not mounted. After editing code, either rebuild or mount the
  workspace and re-run `colcon build` inside the container.

## Editing code without rebuilding

Add to the `sim` service and rebuild once:

```yaml
    volumes:
      - ./auv_simbridge:/ros2_ws/src/auv_simbridge
      - ./auv_scoring:/ros2_ws/src/auv_scoring
      - ./auv_worlds:/ros2_ws/src/auv_worlds
```

then inside the container:

```bash
cd /ros2_ws && colcon build --packages-select auv_simbridge && source install/setup.bash
```

The same trick for the controller is in [../competitor/README.md](../../competitor/README.md).
