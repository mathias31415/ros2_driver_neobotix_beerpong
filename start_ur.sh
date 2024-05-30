#!/bin/bash

echo "Run Container"

##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/ur5-robot/ros2_ws/src
SRC_HOST="$(pwd)"/src
ROS_DISTRO=humble
DEP_CONTAINER=/home/ur5-robot/ws_dependencies
DEP_HOST="$(pwd)"/ws_dependencies



docker run \
  --name ur-driver-humble\
  --rm \
  -it \
  --net=host \
  --env-file .env \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -e DISPLAY="$DISPLAY" \
  ur-driver-humble/ros-render:"$ROS_DISTRO"

  #-v "$DEP_HOST":"$DEP_CONTAINER":rw \
 