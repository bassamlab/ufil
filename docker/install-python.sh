#!/bin/sh
# Pinned Python dependencies of the workspace (docker/requirements.txt + docker/constraints.txt).
# Run as root after install-system.sh. Used by the Dockerfile (`runtime` stage) and by CI.
# Usage: install-python.sh [DIR]   DIR holds requirements.txt and constraints.txt (default: script dir)
set -eu
dir="${1:-$(dirname "$0")}"

# CPU-only torch: the default wheels pull in several GB of CUDA libraries via ultralytics.
# Installed without deps from the PyTorch index; its deps are resolved from PyPI below.
pip3 install --no-cache-dir --break-system-packages \
    --index-url https://download.pytorch.org/whl/cpu --no-deps \
    -c "$dir/constraints.txt" \
    torch torchvision

pip3 install --no-cache-dir --break-system-packages \
    -c "$dir/constraints.txt" \
    -r "$dir/requirements.txt"
pip3 check
