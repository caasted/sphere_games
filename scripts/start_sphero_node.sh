#!/bin/bash

export TEAM=$1
export COLOR=$2
export ADDR=$3

source /etc/profile
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
export ROS_MASTER_URI="http://redteam-node:11311"
export ROS_IP=`hostname -I | cut -d' ' -f1`

python3.5 /home/pi/sphere_games/host/sphero_node.py $TEAM $COLOR $ADDR