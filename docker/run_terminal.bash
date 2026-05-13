#!/usr/bin/env bash

DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
XSOCK=/tmp/.X11-unix

GPU_ARGS=""

# Detect NVIDIA GPU support
if command -v nvidia-smi &> /dev/null || \
   docker info 2>/dev/null | grep -q "Runtimes:.*nvidia"; then
    echo "NVIDIA runtime detected - enabling GPU support"
    GPU_ARGS="--gpus all --runtime=nvidia"
else
    echo "No NVIDIA runtime detected - running without GPU"
fi

docker run -it --rm \
 $GPU_ARGS \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --net=host \
 -v $DIR/../:/workspace/ \
 -v /dev/shm:/dev/shm \
 -v /media/:/media/ \
 -w /workspace/ \
 --entrypoint /bin/bash ufil