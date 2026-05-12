#!/usr/bin/env bash
DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

XSOCK=/tmp/.X11-unix
 
docker run -it --rm \
 --privileged \
 --net=host \
 -v $DIR/../:/workspace/ \
 -v /dev/shm:/dev/shm \
 -w /workspace/ \
 "ufil" \
 /bin/bash -c '
 source /opt/venv/bin/activate
 source /opt/ros/jazzy/setup.bash
 make sequential'  
