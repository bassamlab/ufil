#!/bin/sh
# System and ROS dependencies of the workspace. Run as root on top of ros:jazzy.
# Used by the Dockerfile (`system` stage) and by CI (.gitlab-ci.yml), so both share one list.
set -eu
export DEBIAN_FRONTEND=noninteractive

# Retry flaky mirror downloads instead of failing the whole (long) install
echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries

# Never install recommended or suggested packages, also not in later `apt-get install` or
# `rosdep install` calls (e.g. the devcontainer's postCreate.sh)
printf 'APT::Install-Recommends "false";\nAPT::Install-Suggests "false";\n' \
    > /etc/apt/apt.conf.d/90-no-recommends

apt-get update

# ROS dependencies. Headless: no desktop meta-package and no GUI tools (rviz, rqt); data is
# visualized through foxglove_bridge.
apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-robot-localization \
    ros-jazzy-udp-driver \
    ros-jazzy-udp-msgs \
    ros-jazzy-etsi-its-messages \
    ros-jazzy-lanelet2 \
    ros-jazzy-derived-object-msgs \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-foxglove-bridge

# System libraries and Python tooling (python3-venv is used by `make venv`).
# Python packages that packages declare as rosdep keys come from apt, so `rosdep check` passes.
apt-get install -y --no-install-recommends \
    libtbb-dev \
    libboost-dev \
    libgeographiclib-dev \
    libeigen3-dev \
    libpaho-mqttpp-dev \
    libpaho-mqtt-dev \
    python3-h5py \
    python3-scipy \
    python3-pip \
    python3-venv

rm -rf /var/lib/apt/lists/*
