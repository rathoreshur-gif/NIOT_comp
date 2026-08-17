# Competitor container

The competitor's codebase runs here, isolated from the simulator on its own ROS
domain. This directory is what a team would work in; nothing outside it (and
`auv_msgs`) ends up in the image.

```
sim              ROS_DOMAIN_ID=0    Gazebo, scoring, simulator_bridge
  |
domain_bridge                       whitelist only
  |
competitor       ROS_DOMAIN_ID=1    you are here
```

Because the two sides are separate DDS domains, `ros2 topic list` in this
container shows *only* what the bridge forwards. `/scoring/*`, the ground-truth
flaggers and the raw `/model/auv/*` Gazebo topics are not reachable, by design.

## What you get, and what you can send

Available in this container:

| Topic | Type |
|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` |
| `/localization/pose` | `auv_msgs/msg/AuvState` |
| `/front_camera/image_raw` | `sensor_msgs/msg/Image` |
| `/bottom_camera/image_raw` | `sensor_msgs/msg/Image` |
| `/front_camera/oakd_frame` | `auv_msgs/msg/StereoVisionFrame` |
| `/front_camera/rgbd_frame` | `auv_msgs/msg/RGBDFrame` |

The one topic that reaches the vehicle:

| Topic | Type |
|---|---|
| `/controller/thruster_forces` | `auv_msgs/msg/ThrusterForces` |

Publishing anything else has no effect on the simulation. The whitelist lives in
`../Matsya_ROS2_Simulator/docker/domain_bridge.yaml`.

## Units, which are easy to get wrong

All of this is set by `auv_simbridge/simulator_bridge.py`, and none of it is
what you would guess:

- Positions are in **centimetres** — Gazebo's metres multiplied by 100, relative
  to an origin that `/localization/reset_service` resets.
- Y and Z are **negated** relative to Gazebo. Two flips is a 180° rotation about
  X, so the frame stays right-handed and comes out NED-like: **+z is DOWN**. A
  larger z means deeper, and commanding a negative z asks the vehicle to *rise*.
- Orientation is in **degrees**, not radians — `quaternion_to_euler` calls
  `as_euler('zyx', degrees=True)`. This one is easy to miss and produces a
  controller that looks plausible and diverges.
- `velocity` is in cm/s (also scaled by 100) and is already body-frame.
  `angular_velocity` is in rad/s and is *not* scaled.
- `/controller/setpoint` is interpreted in the same frame and units as the pose:
  centimetres and degrees.
- Thruster forces are newtons, and the hardware saturates at ±40 N.

## Running

From the simulator directory, since it owns the compose file:

```bash
cd ../Matsya_ROS2_Simulator
docker compose up -d              # sim + bridge + competitor
docker compose logs -f competitor
```

A shell inside, for `ros2 topic echo` and friends:

```bash
docker compose exec competitor bash
```

Drive it manually:

```bash
ros2 topic pub --once /controller/setpoint auv_msgs/msg/Pose \
  '{position: {x: 0.0, y: 0.0, z: -100.0}}'      # 100 cm down
```

## The reference controller

`competitor_controller/basic_controller.py` is a deliberately minimal starting
point: PID on the pose error, rotated into the body frame, through a fixed mixer
to the eight thrusters, capped at ±40 N. Replace it.

It controls four axes — surge, sway, heave and yaw. Roll and pitch are left
uncommanded on purpose: the centre of buoyancy sits above the centre of mass, so
the vehicle rights itself passively, and correcting roll/pitch would spend the
same four heave thrusters that are the only source of depth control. The mixer
rows for both axes are still there and correct, so raising `kp`/`kd` on elements
3 and 4 is all it takes to control them if you want to.

The mixer coefficients are derived in closed form from the thruster geometry in
`Matsya_ROS2/auv_controller/config/constants.json` — the four surge thrusters are
a vectored X-configuration at ±45°, the four heave thrusters point straight up —
and were checked against the full control-effectiveness matrix, so each axis
command produces a wrench on that axis alone.

`heave_sign` stays a launch argument rather than a baked-in constant because
`simulator_bridge` negates heave on its way to Gazebo while `AuvState` z is
already sign-flipped — two negations that are easier to confirm by observation
than by reading. Measured on this build the default `+1.0` is correct: commanding
`z: 100.0` descends to 1 m and holds. If that ever changes, flip it:

```bash
ros2 launch competitor_controller competitor.launch.py heave_sign:=-1.0
```

Measured behaviour of the reference gains, from a standing start to `z: 100.0`
with the ocean current enabled: converges in about 20 s with no overshoot, holds
depth within ~1.5 cm and x/y within a few centimetres against the current.

## Editing without rebuilding

Add to the `competitor` service in `../Matsya_ROS2_Simulator/docker-compose.yml`:

```yaml
    volumes:
      - ../competitor/competitor_controller:/ros2_ws/src/competitor_controller
```

then inside the container:

```bash
cd /ros2_ws && colcon build --packages-select competitor_controller \
  && source install/setup.bash
```
