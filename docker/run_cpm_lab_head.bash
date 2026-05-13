#!/bin/bash
DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

WORKSPACE_ON_HOST=$DIR/../
WORSKSPACE_IN_CONTAINER=/workspace/
IMAGE=ufil
CONTAINER=ufil-cpm-lab-head
XSOCK=/tmp/.X11-unix

echo "Running CPM Lab Ufil example in Docker image ${IMAGE} and container name ${CONTAINER}"

docker run --rm --privileged \
 -e DISPLAY=$DISPLAY \
 -v $XSOCK:$XSOCK \
 -v $HOME/.Xauthority:/root/.Xauthority \
 --privileged \
 --network host -v /dev:/dev --name ${CONTAINER} -v ${WORKSPACE_ON_HOST}:${WORSKSPACE_IN_CONTAINER} -w ${WORSKSPACE_IN_CONTAINER} ${IMAGE} /bin/bash -c '
cd /workspace
source install/setup.bash
export ROS_DOMAIN_ID=21
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
ros2 launch ufil_examples cpm-lab_head.launch.xml use_sim_time:=false
' 
