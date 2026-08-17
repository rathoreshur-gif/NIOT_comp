# NIOT competition simulator.
#
# Build context is the repository root, not this directory, so the COPY paths
# below are repo-relative: the image builds five packages out of simulator/ and
# nothing from competitor/. docker-compose.yml already sets `context: .`.
#
#   ./run.sh build
#   docker compose build sim
#
# Jazzy, not Humble: auv_gui builds against the gz_*_vendor packages and the
# world/model SDF targets Gazebo Harmonic, neither of which exists on Humble.
FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Three groups here:
#   - ros-gz + the gz_*_vendor packages auv_gui's CMakeLists find_package()s
#   - Qt5 dev + QML runtime modules, for the Scoreboard/AuvTeleop GUI plugins
#   - the Python deps the simbridge/scoring/worlds nodes import at runtime but
#     never declare in their package.xml, so rosdep alone would not pull them
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
      tmux \
      vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Manifests first, sources second: rosdep re-runs only when a package.xml
# changes, so editing Python leaves the dependency layer cached.
COPY simulator/auv_msgs/package.xml                 src/auv_msgs/package.xml
COPY simulator/auv_gui/package.xml        src/auv_gui/package.xml
COPY simulator/auv_scoring/package.xml    src/auv_scoring/package.xml
COPY simulator/auv_simbridge/package.xml  src/auv_simbridge/package.xml
COPY simulator/auv_worlds/package.xml     src/auv_worlds/package.xml

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

# auv_msgs is vendored into simulator/ so this repo builds on its own; the wider
# team stack it came from is deliberately not part of the competition release.
COPY simulator/auv_msgs                 src/auv_msgs
COPY simulator/auv_gui        src/auv_gui
COPY simulator/auv_scoring    src/auv_scoring
COPY simulator/auv_simbridge  src/auv_simbridge
COPY simulator/auv_worlds     src/auv_worlds

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --event-handlers console_direct+ \
    && rm -rf build log

# This image also serves as the `domain_bridge` compose service: bridging
# auv_msgs topics needs auv_msgs on AMENT_PREFIX_PATH, and the workspace above
# has just built it, so reusing this image beats maintaining a third one.
#
# Deliberately its own layer *after* the colcon build rather than folded into
# the apt block at the top. Adding it up there invalidates every layer below,
# including the ~15 min workspace build, for a package nothing else depends on.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ros-jazzy-domain-bridge \
    && rm -rf /var/lib/apt/lists/*

# Qt refuses to start without a writable XDG_RUNTIME_DIR, and the container
# runs as root, which has none.
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

COPY simulator/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /entrypoint.sh --shell-init" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
