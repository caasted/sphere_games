#!/bin/bash

pushd "$(dirname "$0")"

source set_team_addr.sh

export ROS_MASTER_URI="http://192.168.42.100:11311"
export ROS_IP=`hostname -I | cut -d' ' -f1`

echo "Waiting for roscore to come up... you'll see errors while it waits"
until rosnode info rosout | grep Pid; do sleep 2; done

# Setup Blue Team Node
echo "Attempting to Start Blue Team"
ssh pi@192.168.42.101 "bash -s" < start_sphero_node.sh blue_sphero blue $BLUE_ADDR &

sleep 5

# Setup Red Team Node
echo "Attempting to Start Red Team"
ssh pi@192.168.42.101 "bash -s" < start_sphero_node.sh red_sphero red $RED_ADDR &
sleep 5

echo "Both nodes should be running, if not kill the script, run stop_team.sh, and try again"
sleep 10

python ../host/setup_arena.py -m
popd

echo "Spheros Fully Initialized"

wait

echo "Both Spheros have been stopped"