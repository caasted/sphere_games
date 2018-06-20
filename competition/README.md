# README In Progress

## On the Camera Host (Raspberry Pi 3 B+ w/ Camera Module v2)

Install Raspbian, ROS, and follow build instructions at https://github.com/UbiquityRobotics/raspicam_node

Set the Raspberry Pi to be the ROS Master by adding the following to ~/.bashrc
```
export ROS_MASTER_URI=http://${HOSTNAME}:11311
```

Launch the raspicam node:
```
roslaunch raspicam_node camerav2_1280x720.launch
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
cd [SPHERE_GAMES_REPO_PATH]/competition/
```

Next, on the Raspberry Pi, modify line 7 of "sphero_command.js" to match the Bluetooth device ID of the Sphero SPRK+ paired to the Raspberry Pi. Then run:
```
node sphero_red.js
```

Next, on the Workstation, modify line 7 of "sphero_command.js" to match the Bluetooth device ID of the Sphero SPRK+ paired to the Workstation. Then run:
```
node sphero_blue.js
```

On the workstation:
```
cd
sudo apt install virtualenv python-opencv
mkdir python2_env
virtualenv python2_env
source python2_env/bin/activate
pip install numpy pyyaml rospkg catkin_pkg
```

On the Workstation:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/competition
python red_player.py
```

In a second window:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/competition
python blue_player.py
```

In a third window:
```
source ~/python2_env/bin/activate
cd [path_to_sphere_games]/competition
python display_host.py
```

