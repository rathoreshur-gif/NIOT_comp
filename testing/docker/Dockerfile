# Team testing container.
#
# Not the competition image. This one is a *toolchain*: ROS Jazzy, Gazebo, Qt
# and the Python the team's nodes import, and no source code at all. The
# workspace comes in at runtime as a bind mount of whoever's ~/ros2_ws is
# running it, so a teammate edits on the host and builds in here.
#
#   ./run.sh test_build      build this image
#   ./run.sh test_up         start it with the simulator + simbridge running
#
# Consequences of shipping no sources, both deliberate:
#
#   - the image is built once and stays valid across every code change; only a
#     new *dependency* needs a rebuild (or ./run.sh test_deps for a quick fix)
#   - none of the team's private code ends up baked into an image built out of
#     the public competition repo
#
# Jazzy, not Humble: auv_gui builds against the gz_*_vendor packages and the
# world/model SDF targets Gazebo Harmonic, neither of which exists on Humble.
# It also matches what the team runs natively, so the mounted sources build the
# same way in here as they do outside.
FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The simulator's own dependency set, lifted from simulator/docker/Dockerfile:
#   - ros-gz + the gz_*_vendor packages auv_gui's CMakeLists find_package()s
#   - Qt5 dev + QML runtime modules, for the Scoreboard/AuvTeleop GUI plugins
#   - the Python the simbridge/scoring/worlds nodes import at runtime but never
#     declare in their package.xml, so rosdep alone would not pull them
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-interfaces \
      ros-jazzy-gz-common-vendor \
      ros-jazzy-gz-gui-vendor \
      ros-jazzy-gz-msgs-vendor \
      ros-jazzy-gz-plugin-vendor \
      ros-jazzy-gz-transport-vendor \
      ros-jazzy-cv-bridge \
      ros-jazzy-image-transport \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-compressed-image-transport \
      ros-jazzy-rosbag2-storage-mcap \
      python3-colcon-common-extensions \
      python3-numpy \
      python3-scipy \
      python3-yaml \
      python3-opencv \
      qtbase5-dev \
      qtdeclarative5-dev \
      qtquickcontrols2-5-dev \
      qml-module-qtquick-controls2 \
      qml-module-qtquick-layouts \
      qml-module-qtquick-window2 \
      libgl1-mesa-dri \
      mesa-utils \
      git \
      tmux \
      vim \
    && rm -rf /var/lib/apt/lists/*

# What the *team's* nodes import on top of that (Matsya_ROS2/requirements.txt),
# minus the hardware SDKs. Best-effort as one group rather than strict: these
# are conveniences for running a node by hand, and a package that gets renamed
# in a later Ubuntu should not take the whole image build down with it. Anything
# missing is one `./run.sh test_pip` away at runtime.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
         python3-pandas \
         python3-matplotlib \
         python3-serial \
         python3-zmq \
         python3-openpyxl \
         python3-tk \
         python3-pyqtgraph \
         python3-flask \
       || echo "WARNING: some team extras were not installed; see ./run.sh test_pip" \
    && rm -rf /var/lib/apt/lists/*

# Not packaged for Ubuntu. Noble's pip is PEP 668-managed and this is a
# single-purpose container, so installing into the system interpreter (which is
# the one ROS nodes run under) is the intended thing rather than a venv.
RUN pip install --no-cache-dir --break-system-packages \
      simple-pid \
      ruckig \
    || echo "WARNING: simple-pid/ruckig not installed; see ./run.sh test_pip"

# The vision stack pulls torch and roughly 3 GB with it, which most testing
# does not need. Off by default, on for whoever does:
#
#   ./run.sh test_build --build-arg TEST_VISION_DEPS=true
ARG TEST_VISION_DEPS=false
RUN if [ "$TEST_VISION_DEPS" = "true" ]; then \
      pip install --no-cache-dir --break-system-packages ultralytics; \
    fi

# Seeded here so `./run.sh test_deps` on a workspace with a new package is a
# few seconds rather than a fresh index download.
RUN rosdep update --rosdistro jazzy

# Qt refuses to start without a writable XDG_RUNTIME_DIR, and the container
# runs as root, which has none.
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Build artefacts live outside /ros2_ws on purpose. /ros2_ws is the host's own
# workspace, already carrying a native build/ and install/ from colcon on the
# host; writing container builds into those would leave one tree holding two
# incompatible sets of artefacts. This path is a named Docker volume instead,
# so it persists between runs and stays invisible to the host build.
ENV TEST_WS_BASE=/opt/testws
RUN mkdir -p /opt/testws

COPY testing/docker/entrypoint.sh /entrypoint.sh
COPY testing/docker/build_ws.sh   /usr/local/bin/build_ws.sh
COPY testing/docker/start_sim.sh  /usr/local/bin/start_sim.sh
RUN chmod +x /entrypoint.sh /usr/local/bin/build_ws.sh /usr/local/bin/start_sim.sh \
    && echo "source /entrypoint.sh --shell-init" >> /root/.bashrc

WORKDIR /ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
