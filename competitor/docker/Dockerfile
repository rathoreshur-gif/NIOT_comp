# The competitor's container.
#
# Build context is the repository root, not this directory: the controller needs
# auv_msgs, which lives under simulator/. docker-compose.yml already sets
# `context: .`.
#
#   docker compose build competitor
#
# ros-base, not desktop: this container never renders anything and never runs
# Gazebo, so it needs neither the gz stack nor Qt. That makes it a fraction of
# the size of niot-sim, and it deliberately contains no simulator or scoring
# code at all — the competitor's view of the world is whatever crosses the
# domain bridge.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-numpy \
      vim \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# Manifests first, sources second: rosdep re-runs only when a package.xml
# changes, so editing the controller leaves the dependency layer cached.
COPY simulator/auv_msgs/package.xml                      src/auv_msgs/package.xml
COPY competitor/competitor_controller/package.xml          src/competitor_controller/package.xml

RUN apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/*

COPY simulator/auv_msgs                                  src/auv_msgs
COPY competitor/competitor_controller                      src/competitor_controller

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --event-handlers console_direct+ \
    && rm -rf build log

COPY competitor/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo "source /entrypoint.sh --shell-init" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
