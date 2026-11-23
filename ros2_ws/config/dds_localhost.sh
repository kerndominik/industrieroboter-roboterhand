#!/bin/bash
# Source this file to run all ROS 2 nodes of this computer with CycloneDDS over
# the loopback interface (recommended for WSL 2, see README "WSL notes"):
#
#     source ros2_ws/config/dds_localhost.sh
#
# Needs: sudo apt install ros-humble-rmw-cyclonedds-cpp
_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${_dir}/cyclonedds_localhost.xml"
export ROS_LOCALHOST_ONLY=0   # the CycloneDDS config already restricts to "lo"
unset _dir
