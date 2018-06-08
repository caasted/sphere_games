# README In Progress

## On the Camera Host (Raspberry Pi 3 B+ w/ Camera Module v2)

Follow build instructions at https://github.com/UbiquityRobotics/raspicam_node

At the step "Compile the code with catkin_make," perform the following steps:
```
mkdir build
cd build
cmake ../
make
```

Set the Raspberry Pi to be the ROS Master:
```
export ROS_MASTER_URI=http://${HOSTNAME}:11311
```

Launch the raspicam node:
```
roslaunch raspicam_node camerav2_1280x720.launch
```

## On the Display Host (Workstation w/ NVIDIA GPU)

Execute the following command, inserting the hostname of the Camera Host Raspberry Pi:
```
export ROS_MASTER_URI=http://[Raspberry Pi Hostname]:11311
```

