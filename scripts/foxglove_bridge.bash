#! /bin/bash

sudo apt-get update
sudo apt-get install -y ros-$ROS_DISTRO-foxglove-bridge

ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090

rosbridge_address