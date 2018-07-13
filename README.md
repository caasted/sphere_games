# sphere_games

ROS/Gazebo simulations of sphere robots for reinforcement learning activities and ROS/JavaScript communication for Sphero SPRK+ robots.

Designed in Ubuntu 16.04 using ROS Kinetic and Gazebo 7.

# Shared Instructions

## Installation
Make sure the following commands have been added to your .bashrc file or ran in the terminal used to launch rosrun:
```
source /opt/ros/kinetic/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:[INSERT_PATH_TO_REPO]/plugins/build
```

Create and configure a virtual environment for managing Python packages:
```
sudo apt install virtualenv python-opencv
mkdir python2_env
virtualenv python2_env --python=python2.7
source python2_env/bin/activate
pip install numpy pyyaml rospkg catkin_pkg getkey
```

# Simulation Instructions

## Build
In a terminal,
```
cd [REPO_PATH]/plugins/
mkdir build
cd build
cmake ../
make
```

## Execution
In a terminal, run `roscore`

In a second terminal,
```
cd [REPO_PATH]/arenas/
source /usr/share/gazebo/setup.sh
rosrun gazebo_ros gazebo ctf_1v1_arena.world
```

In a third terminal, run the simulation tracker python script:
```
source ~/python2_env/bin/activate
cd [repo_path]/host/
python sim_tracker.py
```

In a fourth terminal, run the reinforcement learning agent script:
```
source ~/python2_env/bin/activate
cd [repo_path]/agents/
python red_learning_agent.py
```

In a fifth terminal, run the simple proportional control agent script:
```
source ~/python2_env/bin/activate
cd [repo_path]/agents/
python blue_simple_agent.py
```

The red and blue spheres will both attempt to reach the other teams 'flag' and return to their own base as quickly as possible.

# Physical Robot Instructions

## On the Camera Host (Raspberry Pi 3 B+ w/ Camera Module v2)

Install Raspbian, ROS, and follow build instructions at https://github.com/UbiquityRobotics/raspicam_node

Set the Raspberry Pi to be the ROS Master by adding the following to ~/.bashrc
```
export ROS_MASTER_URI=http://${HOSTNAME}:11311
```

Launch the raspicam node:
```
roslaunch raspicam_node camerav2_1280x960.launch
```

## On the Display Host (Workstation w/ NVIDIA GPU)

Add the following line to the ~/.bashrc to point to the Raspberry Pi for the ROS master. Insert the hostname of the Camera Host Raspberry Pi:
```
export ROS_MASTER_URI=http://[Raspberry Pi Hostname]:11311
```

## On the Each Host Computer (Raspberry Pi and Workstation)

Establish a bluetooth connection with each Sphero, one per computer.
```
cd
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt install nodejs
git clone https://github.com/orbotix/sphero.js
cd sphero.js
npm install noble
sudo chown -R ${USER}:${USER} /usr/lib/node_modules
npm install -g
npm install rosnodejs -g
```

At this point we need to launch a node and let it fail before we can complete the setup process. First, modify line 4 in examples/color.js to contain the Bluetooth device ID of your Sphero.
```
Replace var orb = sphero(process.env.PORT);
With var orb = sphero("D6:DA:83:63:D0:2B");
Where the Bluetooth device ID matches your own.
```

Next launch color.js using node:
```
cd examples
node color.js
```

This should produce the following error:
```
noble warning: adapter state unauthorized, please run as root or with sudo
               or see README for information on running without root/sudo:
               https://github.com/sandeepmistry/noble#running-on-linux
```
Press ```Control-C``` to exit and then the following command to correct it:
```
sudo setcap cap_net_raw+eip $(eval readlink -f `which node`)
```

On both computers:
```
cd [SPHERE_GAMES_REPO_PATH]/host/
```

Next, on the Raspberry Pi, modify line 7 of "red_node.js" to match the Bluetooth device ID of the Sphero SPRK+ paired to the Raspberry Pi. Then run:
```
node red_node.js
```

In a new terminal, calibrate the internal Sphero heading by launching the manual control script and controlling the Sphero with the arrow keys:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/calibrate/
python manual_control.py 1
```
```
Controls:
Up: Increase speed
Down: Decrease speed
Left: Rotate counterclockwise
Right: Rotate clockwise
'q': Set current heading to zero position for Sphero.
```
The zero heading position should be set to match the camera reference frame (facing camera right).

Next, on the Workstation, modify line 7 of "blue_node.js" to match the Bluetooth device ID of the Sphero SPRK+ paired to the Workstation. Then run:
```
node blue_node.js
```

In a new terminal, calibrate the internal Sphero heading by launching the manual control script and controlling the Sphero with the arrow keys:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/calibrate/
python manual_control.py 0
```

In a new terminal on the workstation:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/host
python sphero_tracker.py
```

In a second window:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/agents
python red_learning_agent.py
```

In a third window:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/agents
python blue_simple_agent.py
```

