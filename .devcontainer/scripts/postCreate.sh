#!/usr/bin/env bash
set -e

echo "[ROS] Building..."

rosdep update

sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*
sudo apt-get update

sudo PIP_BREAK_SYSTEM_PACKAGES=1 rosdep install \
  --from-paths src --ignore-src -y \
  --skip-keys 'python3-ultralytics-pip'
echo "[ROS] Build complete."
