# Gamepad demo mode

Drive the competition simulator by hand from a gamepad, for showing it at an
event. Nothing here changes the competition path: `./run.sh` and
`basic_controller` behave exactly as they did.

```bash
./demo.sh up        # start the stack in gamepad mode
./demo.sh log       # follow the controller
./demo.sh down      # stop
```

## Controls

The left stick is **in the vehicle's frame**, like a game: forward is wherever
the nose is pointing, not down the pool.

| | |
|---|---|
| **Left stick** | move — forward/back along the nose, left/right strafes |
| **Right stick X** | swing the nose left / right |
| **Right stick Y** | rise / dive |
| **RT** | boost, analogue — squeeze for more speed |
| **LT** | precision, analogue — squeeze for fine control |
| **A** | drop the marker — press again to pick it back up |
| **Y** | fire the torpedo — press again to reload it |
| **X** | gripper: toggle open / closed |
| **LB** | anchor here |
| **L1+L2+R1+R2** | squeeze all four: next gear — SLOW → NORMAL → FAST → TURBO |
| **D-pad up/down** | one gear up / down (same four, clamped at the ends) |
| **D-pad left** | back to NORMAL |
| **START** | start the run |
| **BACK** | end the run |

Reset is **not** on the pad. It re-poses the vehicle and re-seeds the whole
course, so it belongs to whoever is running the stand, not to a thumb halfway
through a run: `./demo.sh reset`.

### Scoring

Every one of these pays **once**. Reloading the marker or the torpedo is free
and unlimited, so paying per drop or per shot made hovering over one bin the
highest-scoring strategy on the course.

| | Points | Paid |
|---|---|---|
| **Gate** | 50 | once, either half — left and right are worth the same |
| **Slalom** | 100 | per layer, either side of the red pole, each layer once |
| **Bins** | 500 | per bin, each of the four bins once |
| **Torpedo** | 500 | per hole, each of the four holes once |
| **Octagon** | 1000 | per object delivered onto either side pad, each object once |

So the course is worth 50 + 300 + 2000 + 2000 + 4000 = **8350**, plus the time
bonus. Four shots through the same hole is one skill done four times; four
shots through four holes is the task.

The octagon is the big prize because grasping an object and placing it is the
hardest thing the vehicle does. Delivery means the object is inside a side pad's
footprint, resting on its surface, and has **stopped moving** for half a second
— carrying one over the pad in the jaw does not count, and neither does one
bouncing across it.

Every number lives under `tasks.<task>.scoring.points` in
`competition_config.yaml`, which is bind-mounted, so changing them is an edit
and `docker compose restart sim`.

### Speed modes

Four gears, and the current one is on the scoreboard in the Gazebo window so
the driver and everyone watching can see it:

| | | |
|---|---|---|
| **SLOW** | 0.47 m/s | a first-timer, or a crowded table |
| **NORMAL** | 0.88 m/s | the default — enough to get about, hard to crash |
| **FAST** | 1.15 m/s | |
| **TURBO** | 1.40 m/s | everything the thrusters have |

Squeeze **L1 + L2 + R1 + R2 together** to step to the next one; it wraps round
from TURBO back to SLOW. A two-handed chord on purpose: this is the one control
that changes how every other one behaves, so it should be impossible to hit by
accident and unmistakable when it is meant. The D-pad does the same thing one
gear at a time for anyone who would rather not, and D-pad left goes back to
NORMAL.

TURBO is not an arbitrary number. Four surge thrusters at 45° with the demo's
75 N ceiling give 212 N of forward force, and against the hull's own drag curve
that solves to 1.40 m/s. Asking for more does not go faster — it parks the
velocity loop in saturation and takes the yaw authority down with it, because
the same four thrusters do both.

While the triggers are squeezed, **LB does not anchor** — it is half of the gear
chord.

### Payload

One marker, one torpedo, one gripper — all on the vehicle's centreline.

**A and Y toggle.** Press once to drop or fire; press again and the payload is
teleported back into its cradle and re-welded to the hull, ready for another go.
That is what makes the stand playable: a child can shoot the board, miss, press
Y, and shoot again without anyone restarting Gazebo. Reloading no longer earns
repeat points, though — see the scoring table below.

Firing is a *launch*: simulator_bridge releases the tube and then applies a
one-step impulse along the nose, so the torpedo runs about 3 m and coasts to a
stop. Aim by pointing the vehicle — the shot goes wherever the nose is looking
when Y is pressed.

The gripper only welds a pickup on an explicit close, so brushing past an object
with the fingers open never picks it up — X is the whole grabbing mechanism.

The marker and the torpedo are **drawn at 3× and orange** so they read from
across a room. Only the visual is scaled: mass, buoyancy and every collision
come off a 1× collision mesh, so the marker falls exactly as it always did and
the torpedo is still the 24 mm that goes cleanly through the board's ~97 mm
holes. Scale the collision too and an off-centre shot starts clipping the rim.

**Let go of every stick and it freezes** — position, heading and depth are
pinned where they were and the controller brakes back to them. Nothing coasts
and nothing floats up, which is what makes it drivable in front of a crowd.

## The run has to be started

The thrusters are dead until the run starts — that is the competition
lifecycle, not a fault. **Press START.** The sticks do nothing before that, and
the log says so the first time it is tried.

While the run is idle the vehicle drifts up on its own buoyancy. The controller
re-anchors every tick while idle, so START always begins holding wherever it has
drifted to rather than hauling back to a boot-time pose.

Between competitors, `./demo.sh reset` re-poses the vehicle, re-seeds the course
and clears the score. It is a stand-operator command rather than a pad button on
purpose. A run that reaches its time limit resets itself: with `auto_start` on
(the default) the next person to push a stick re-seeds and starts, so the demo
loops without anyone touching the keyboard.

To skip the lifecycle entirely and have the thrusters live from boot:

```bash
SIM_LIFECYCLE=false ./demo.sh up
```

## Knobs

```bash
DEMO_SPEED=0.30 ./demo.sh up          # slower, for a crowded table
JOY_DEV=/dev/input/js1 ./demo.sh up   # pin a particular pad
DEMO_HEAVE_SIGN=-1.0 ./demo.sh up     # if it rises when told to dive
SIM_GUI=false ./demo.sh up            # headless
```

Everything else is a live ROS parameter, retunable without a restart:

```bash
./demo.sh shell
ros2 param set /gamepad_controller cruise_speed 0.6
ros2 param list /gamepad_controller
```

## The pad

Written against a Zebronics pad in **XInput mode**, which the kernel enumerates
as `Microsoft X-Box 360 pad` — 8 axes, 11 buttons. If the pad has a mode switch
(usually on the back, sometimes HOME held for a few seconds) it has to be on
XInput; in DirectInput mode the axis numbers differ and the sticks will do the
wrong things.

```bash
./demo.sh check     # is the pad there, and on which node
```

### Worn sticks are calibrated automatically

The stick centres are **measured, not assumed**. The kernel reports every axis's
current value the moment the device is opened, so the first read after a
connect is where that pad's sticks actually rest — and on a pad that has been
through a few hundred hands, that is not zero. The one this was developed
against rests its right stick X at **−0.55 of full deflection**, which through
the deadzone and the expo curve is a standing yaw command of 0.14 rad/s: the
vehicle turns on its own, forever, with the pad face down on the table.

The measured offset is subtracted, and each side of the centre is scaled to its
own remaining travel, so full deflection still reaches 1.0 in both directions.
It is logged on connect and published in `./demo.sh status` as `stick_centres`.

Calibration happens on **every connect**, so a pad that starts misbehaving is
fixed by unplugging it and plugging it back in. Do not hold the sticks while
doing that — an offset past 0.80 is refused rather than stored, and logged, on
the assumption that somebody's thumb is on it.

The pad is read straight from `/dev/input/js*` with the Linux joystick API, not
through the `joy` package, so the competitor image needs no extra dependency.
`/dev/input` is bind-mounted into the container, so **a pad unplugged mid-demo
is safe**: the vehicle freezes, and plugging it back in reconnects on its own
without restarting anything — including onto a different `js*` number, which is
what the kernel usually does on a re-plug.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Sticks do nothing, vehicle just sits | The run has not started — press START |
| Vehicle rises when told to dive | `DEMO_HEAVE_SIGN=-1.0 ./demo.sh up` |
| Sticks do the wrong things | Pad is in DirectInput mode; switch it to XInput |
| `no joystick at /dev/input/js0` | `./demo.sh check`, then `JOY_DEV=...` |
| Vehicle drifts slowly while parked | Expected: the hold has a 2 cm deadband |
| Vehicle turns / creeps with hands off | A stick resting off centre. Re-plug the pad — hands off — and check the `Pad stick centres` line in `./demo.sh log` |
| Low real-time factor | Expected — see `simulator/docker/PERFORMANCE.md` |

## How it works

`competitor/competitor_controller/competitor_controller/gamepad_controller.py`
runs in the competitor container on `ROS_DOMAIN_ID=1`, exactly where a
competitor's code runs, and is allowed exactly what a competitor is allowed:
`/controller/thruster_forces` out and `/simulator/run_control` for the
lifecycle. The simulator, the scoring and the domain bridge are untouched.

Control is the same cascade `auv_simbridge/teleop.py` uses for keyboard
driving, because it is already tuned for this hull: the sticks ask for a
body-frame velocity, each axis turns that into a force with the model's own drag
curve fed forward plus PI on the velocity error, and the resulting
(Fx, Fy, Fz, Mz) is split across the eight thrusters through the real allocation
matrix. Speed ceilings are solved against the drag curves for 40 N thrusters —
about 0.81 m/s surge, 0.72 rad/s yaw, 0.54 m/s heave — because commanding a
speed the vehicle cannot reach just parks the loop in saturation.

`basic_controller` and this node both publish `/controller/thruster_forces`, so
only one may run at a time. That is why demo mode is a compose overlay that
*replaces* the competitor command rather than adding to it.
