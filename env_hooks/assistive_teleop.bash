#! /bin/bash

source /usr/share/gazebo/setup.bash
export GAZEBO_MODEL_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/assistive_teleop/models/:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="${CATKIN_ENV_HOOK_WORKSPACE}/../src/assistive_teleop/models/:${CATKIN_ENV_HOOK_WORKSPACE}/../src/assistive_teleop/worlds/:${GAZEBO_RESOURCE_PATH}"
