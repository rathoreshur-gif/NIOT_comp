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

# What the *team's* nodes import on top of that: every third-party module
# Matsya_ROS2 imports, from apt wherever Ubuntu packages it. Strict on purpose.
# This used to be best-effort, and a missing `pip` let the pip step below fail
# too, so the image shipped without simple-pid and `auv_controller` would not
# start. The import check at the end of this file now fails the build instead.
#
#   python3-pip        pip itself; osrf/ros:jazzy-desktop does not include it
#   python3-pyqt5      the Qt binding pyqtgraph needs (auv_map gui_builder)
#   libusb-1.0-0       what depthai loads when it is imported
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip \
      python3-pandas \
      python3-matplotlib \
      python3-serial \
      python3-zmq \
      python3-openpyxl \
      python3-tk \
      python3-pyqt5 \
      python3-pyqtgraph \
      python3-flask \
      python3-pytest \
      libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Anything pip installs from here on, in this build or later from
# `./run.sh test_shell`, has to leave apt's numpy alone. rclpy, cv_bridge and
# the apt scipy/pandas/opencv are all built against numpy 1.26. If a wheel pulled
# in numpy 2 on top of it, they would break at import time. PIP_CONSTRAINT makes
# every pip invocation respect that, not just the ones in this file. Noble's pip
# is PEP 668-managed. This is a single-purpose container, so installing into the
# system interpreter (the one ROS nodes run under) is the intended thing.
RUN python3 -c 'import numpy; print("numpy==" + numpy.__version__)' > /etc/pip-constraints.txt
ENV PIP_CONSTRAINT=/etc/pip-constraints.txt \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PIP_ROOT_USER_ACTION=ignore

# Not packaged for Ubuntu.
#   simple-pid   auv_controller (controller_interpolate, custom_pid, pid_control)
#   ruckig       auv_navigator (navigatorServer)
#   depthai 3.x  auv_vision and the OAK camera drivers. The code uses the v3 API
#                (dai.node.Camera, ImageAlign), so v2 will not work. Importing
#                it needs no camera attached.
#   nidaqmx      auv_acoustics. Pure Python, so it imports without NI's driver.
#                Only talking to a real DAQ needs that.
RUN pip install --no-cache-dir \
      'simple-pid>=2.0.0' \
      ruckig \
      'depthai>=3,<4' \
      nidaqmx

# auv_vision's `vision` node imports ultralytics (and torch behind it) at start.
# On by default so every node runs, but with the CPU-only torch wheel. That is
# a few hundred MB, where the default CUDA build would be ~3 GB.
#
#   --build-arg TEST_VISION_DEPS=false             leave the vision stack out
#   --build-arg TORCH_INDEX=https://download.pytorch.org/whl/cu124   CUDA torch
#
# ultralytics depends on the opencv-python wheel. That wheel shadows apt's cv2,
# and it bundles its own Qt, which then breaks every PyQt5 window that imports
# cv2. So the wheel is removed again right away, and apt's python3-opencv (which
# cv_bridge is built against) stays the only cv2.
ARG TEST_VISION_DEPS=true
ARG TORCH_INDEX=https://download.pytorch.org/whl/cpu
RUN if [ "$TEST_VISION_DEPS" = "true" ]; then \
      pip install --no-cache-dir --index-url "$TORCH_INDEX" torch torchvision \
      && pip install --no-cache-dir 'ultralytics>=8.3.170' \
      && pip uninstall -y opencv-python opencv-python-headless; \
    fi

# Fail the build here, not in a teammate's `ros2 run` a week later. This is
# every third-party module a Matsya_ROS2 node imports. cv_bridge lives under
# /opt/ros, hence the source.
RUN source /opt/ros/jazzy/setup.bash \
    && python3 -c 'import simple_pid, ruckig, depthai, nidaqmx, numpy, scipy, \
      pandas, matplotlib, serial, zmq, openpyxl, tkinter, pyqtgraph, flask, cv2, \
      cv_bridge; from pyqtgraph.Qt import QtWidgets' \
    && if [ "$TEST_VISION_DEPS" = "true" ]; then \
         python3 -c 'import torch, ultralytics, cv2, numpy; \
           assert numpy.__version__.startswith("1."), numpy.__version__; \
           assert cv2.__file__.startswith("/usr/lib/"), cv2.__file__'; \
       fi

# Seeded here so `./run.sh test_deps` on a workspace with a new package is a
# few seconds rather than a fresh index download. Non-fatal: it is only a
# cache, and some networks block raw.githubusercontent.com outright (it fails
# with SSL: WRONG_VERSION_NUMBER). Building the image should not depend on that.
# test_deps retries it when the cache is empty.
RUN rosdep update --rosdistro jazzy \
    || echo "WARNING: rosdep update failed; ./run.sh test_deps will retry it"

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

# Matsya_ROS2 finds its config and model files by absolute path, not through
# the share directory. About 25 files use ~/ros2_ws/src/Matsya_ROS2/... (for
# example auv_controller's constants.json), and the robot-side ones use
# /home/matsya/ros2_ws. In here ~ is /root and the workspace is mounted at
# /ros2_ws, so both paths are pointed at the mount. Otherwise the controller
# dies at start with FileNotFoundError, even though it built and imported fine.
RUN ln -s /ros2_ws /root/ros2_ws \
    && mkdir -p /home/matsya \
    && ln -s /ros2_ws /home/matsya/ros2_ws

WORKDIR /ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
