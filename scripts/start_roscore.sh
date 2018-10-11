#!/bin/bash

source /etc/profile
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
export ROS_MASTER_URI="http://redteam-node:11311"
export ROS_IP=`hostname -I | cut -d' ' -f1`

/opt/ros/kinetic/bin/roscore