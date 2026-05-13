#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

WORKSPACE_ON_HOST=$SCRIPT_DIR/../
WORSKSPACE_IN_CONTAINER=/workspace/
IMAGE=ufil
CONTAINER=ufil--router

echo "Starting zenoh router with Docker image ${IMAGE} and container name ${CONTAINER}"
trap "docker kill ${CONTAINER}" 0
docker run \
--rm \
--privileged \
--network host \
--name ${CONTAINER} \
-v /dev/:/dev/ \
-v ${WORKSPACE_ON_HOST}:${WORSKSPACE_IN_CONTAINER} \
-w ${WORSKSPACE_IN_CONTAINER} \
${IMAGE} \
/bin/bash -c 'ros2 run rmw_zenoh_cpp rmw_zenohd'
