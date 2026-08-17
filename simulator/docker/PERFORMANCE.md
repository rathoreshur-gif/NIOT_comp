# Simulator performance: measurements and tuning

All figures measured on this machine (i5-13420H, 12 threads, RTX 4050 Laptop,
Ubuntu 24.04) with `gz topic -e -t /world/underwater_pool/stats`, which is
Gazebo's own RTF, averaged over 6 samples after a ~60 s warm-up.

## The headline: the container is not the problem

Like-for-like, container and host perform the same. What changed between the
"RTF 1" and "RTF 0.25" observations was **`simulator_bridge`**, not Docker.

| Scenario (untuned) | Host | Container |
|---|---|---|
| Headless + simulator_bridge | ~0.42 | ~0.40 |
| GUI + teleop + simulator_bridge | ~0.38 | ~0.30 |
| GUI + teleop, **no** simulator_bridge | **0.45–1.38** | 0.76–0.86 |

That last row is where "RTF ≈ 1" came from: without `simulator_bridge`
subscribing to the camera streams. Starting it costs roughly half the RTF on
host and in the container alike.

The container is modestly (~20 %) slower **only** with the Gazebo GUI running,
for the GPU reason below. Headless, the two are indistinguishable.

## GPU status

The host runs Gazebo on the RTX 4050 (`nvidia-smi` shows `gz sim server`
≈1009 MiB and `gz sim gui` ≈983 MiB, ~38 % utilisation).

The container **cannot**. It sees the NVIDIA device node but has no driver
userspace, so EGL falls back to the Intel iGPU:

```
libEGL warning: pci id for fd 45: 10de:28a1, driver (null)
```

`10de:28a1` is the RTX 4050. The fix is the NVIDIA Container Toolkit, which is
**not installed** on this machine (`nvidia-container-toolkit`: none; candidate
1.20.0-1). Docker has a stale `nvidia` runtime registration without the binary,
so adding `runtime: nvidia` currently fails with:

```
exec: "nvidia-container-runtime": executable file not found in $PATH
```

To enable it:

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

then add `runtime: nvidia` to the `sim` service in `docker-compose.yml`.

Worth doing if you want the GUI inside the container, but note it is **not**
where the big win is — see below.

## Where the time actually goes

CPU never exceeded ~67 % of 12 threads in any run; 33–50 % of the machine sat
idle at RTF 0.3. This is a **serial** bottleneck (Gazebo's sensor/physics
update loop), not a lack of cores. Throwing more CPU at it will not help;
reducing work on the critical path will.

Two causes dominate, both in the simulation config rather than in Docker.

### 1. Camera sensors — the larger cost

`models/m7urdfnew_sdf_package/model.sdf` defines two cameras, both
**1400×800 @ 30 Hz**: `front_camera` (an `rgbd_camera`, so colour + depth +
point cloud) and `bottom_camera`. Measured traffic:

| Topic | Message size | Rate | Subscribers |
|---|---|---|---|
| `/model/auv/front_camera/points` | **26.88 MB** | 62 MB/s | **none** |
| `/model/auv/front_camera/image` | 3.36 MB | 50 MB/s | simulator_bridge |
| `/model/auv/front_camera/depth_image` | ~3.4 MB | ~50 MB/s | simulator_bridge |
| `/model/auv/bottom_camera/image` | 3.36 MB | ~50 MB/s | simulator_bridge |

`/points` is generated, serialised and bridged into ROS for **nobody** —
`simulator_bridge.py` never subscribes to it (see its subscriber list around
lines 101–132). Removing it from `config/bridge_topics_thrusters.yaml` saves
62 MB/s of pure waste.

Note: dropping `/points` alone did **not** move RTF (0.353 → 0.354). It is
bandwidth and memory waste, not the critical path. Resolution is what matters.

### 2. Physics — 1 ms steps with 50 solver iterations

`worlds/robosub.sdf` uses `<max_step_size>0.001</max_step_size>` (1000 Hz) with
`<iters>50</iters>`. Both are aggressive; 2 ms / 20 iters is a common choice.

**Use 2 ms, not 4 ms.** Measured: 4 ms with 20 iters made things *worse* at
full camera resolution (0.30 vs 0.41 baseline), consistent with the larger step
destabilising the buoyancy/hydrodynamics solve and generating extra contact
work. 2 ms was a clean win.

## Measured gains

Uncapped (`<real_time_factor>0</real_time_factor>`) so the numbers show real
headroom rather than clamping at the world's 1.0 ceiling. Headless +
`simulator_bridge`.

| Config | Max RTF | vs baseline |
|---|---|---|
| Baseline | 0.301 | — |
| Physics only (2 ms, 20 iters) | 0.503 | 1.67× |
| Camera only (640×480, 15 Hz, no `/points`) | 0.904 | 3.0× |
| Cameras effectively disabled | 0.983 | 3.3× |
| **Both** | **1.455** | **4.8×** |

With the ceiling back at 1.0, the tuned config holds a **flat 1.000 at ~40 %
CPU** — i.e. ~31 % slack, which is what keeps RTF stable when the scene gets
busy. That is the answer to "I want a high RTF so I get a stable 1.0".

## What was applied, and what it achieved

All four config changes below are **now in the repo**. Measured after applying
them, headless + `simulator_bridge`:

| Environment | Before | After |
|---|---|---|
| Host, headless + bridge | ~0.42 | **0.75–1.00** (pinned at 1.0, dips are thermal) |
| Container, headless + bridge | ~0.40 | **0.46–1.04** (same, dips are thermal) |
| Container, GUI + teleop + bridge | ~0.30 | ~0.45, still unstable |

Verified alongside the RTF numbers: the Gazebo camera now publishes 700×400, the
rectified topic competitors consume (`/front_camera/image_raw`) is **unchanged at
1280×720**, `/model/auv/front_camera/points` is no longer bridged, and
`simulator_bridge` logs no errors.

The container with the GUI is still the weak case — that is the iGPU fallback
described above, not the simulation config. Run `gui:=false`, or install the
NVIDIA Container Toolkit.

Chosen resolution is **700×400**, not the 640×480 used in the benchmarks: it is
exactly half of 1400×800, so the aspect ratio and vertical FOV are preserved and
the focal length halves exactly. It is also marginally fewer pixels than
640×480, so the measured gains hold.

## The changes

Ranked by payoff. None of these are Docker-specific — they help on host too.

**1. Halve camera resolution** (`models/m7urdfnew_sdf_package/model.sdf`, both
sensors). Biggest single win.

```xml
<width>700</width>    <!-- was 1400 -->
<height>400</height>  <!-- was 800  -->
```

This is coupled to `simulator_bridge._init_distortion_maps`, which hardcodes the
source geometry. Both were updated together — if you change the resolution again
you must change these too, or rectification silently samples the wrong pixels:

```python
def create_map(K, D, f_sim, out_w=1280, out_h=720, sim_w=700, sim_h=400):
...
create_map(front_K,  front_D,  380.62)   # was 761.24
create_map(bottom_K, bottom_D, 376.30)   # was 752.59
```

**2. Drop the unused point cloud** — the `/model/auv/front_camera/points` block
is gone from `config/bridge_topics_thrusters.yaml`. Note this is a footgun fix,
not a speed fix: the entry was `lazy: true`, so it cost nothing while
unsubscribed, and removing it did not change RTF. What it prevents is a casual
`ros2 topic echo` pulling 26.9 MB messages and stalling the machine.

**3. Loosen physics** (`worlds/robosub.sdf`):

```xml
<max_step_size>0.002</max_step_size>  <!-- was 0.001 -->
<iters>20</iters>                     <!-- was 50    -->
```

**4. Camera rate 30 → 15 Hz** — applied to the two camera sensors only. The IMU
stays at 30 Hz and the contact sensors at 30 Hz; do not blanket-replace
`<update_rate>30</update_rate>` in that file or you will silently halve the IMU
rate too.

**5. Run headless when you do not need to watch** — `gui:=false`. In the
container the GUI is the remaining bottleneck once 1–4 are applied (tuned
headless 1.0+, tuned with GUI ~0.35), because it renders on the iGPU.

**6. Install the NVIDIA Container Toolkit** (above) if you want the GUI in the
container at full speed.

### Caveats

Changes 1 and 4 alter what competitors' vision code receives — the rectified
image is still 1280×720, but it is now upsampled from a 700×400 render, so fine
detail is genuinely reduced and detection ranges may shorten. Change 3 alters the
dynamics slightly. These are competition-semantics decisions, not just perf
knobs: **validate scoring behaviour on a full run before using them for a real
competition.** Reverting any one of them is independent of the others.

`config/bridge_topics.yaml` and `config/mini_bridge_topics_thrusters.yaml` still
bridge `/points`; they belong to the other launch files and were left alone.

## Thermal throttling

The package temperature (`x86_pkg_temp`) sits at ~72 °C idle and hits **95 °C**
under simulation load, where the CPU throttles. This is a large source of the
run-to-run RTF variance seen above (the same config measured 0.451 and 0.376 on
consecutive runs), and it is why back-to-back benchmarking needs a cooldown gap.

Ignore `thermal_zone0` (`acpitz`) — it reports a constant bogus 95 °C on this
laptop. Use `thermal_zone3` (`x86_pkg_temp`).

Reducing the compute per the recommendations above is also the most effective
way to keep clocks up and RTF steady; a cooling pad and the `performance`
governor help secondarily.
