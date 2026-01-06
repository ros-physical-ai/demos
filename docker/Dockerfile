# Author: Vu Anh Vu <anh.vu.vu@huawei.com>
#
# Docker image for ROS Physical AI Demos.
# Build: docker buildx build . -t pai_demo --load

ARG DISTRO=kilted
FROM ros:${DISTRO}

# Avoid interactive prompts during apt installs
ENV DEBIAN_FRONTEND=noninteractive

# Use bash for RUN so we can safely 'source' ROS setup scripts
SHELL ["/bin/bash", "-c"]

# ---- Common build tooling ----
RUN apt-get update \
  && apt-get upgrade -y \
  && apt-get install -y --no-install-recommends \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  git \
  build-essential

# ---- Workspace layout ----
RUN mkdir -p /ws_pai/src
WORKDIR /ws_pai

RUN git clone --depth 1 https://github.com/ros-physical-ai/demos src/demos

# ---- Dependency install (optional, but typical) ----
RUN vcs import src < src/demos/pai.repos --recursive
RUN rosdep update \
  && apt-get update \
  && rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -yir

# ---- Build the workspace ----
RUN source /opt/ros/"$ROS_DISTRO"/setup.bash \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---- Cleanup ----
RUN apt-get clean && rm -rf /var/lib/apt/lists/* \
  && find /ws_pai -type d -name ".git" -exec rm -rf {} +

# ---- Runtime environment ----
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp