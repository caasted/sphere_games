# ROS Comms
Exposes basic Sphero interface as a ROS node, allowing control via rospy.

## Installation
Connect to your Sphero SPRK+ via Bluetooth, then modify line 7 of "sphero_command.js" to match the Bluetooth device ID of your own Sphero SPRK+.

Install Node.JS, Sphero.JS, and rosnodejs:
```
cd ~/
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt install nodejs
git clone https://github.com/orbotix/sphero.js
cd sphero.js
npm install noble
sudo mkdir /usr/lib/node_modules
sudo chown -R ${USER}:${USER} /usr/lib/node_modules
npm install -g
npm install rosnodejs -g
```

At this point we need to launch a node and let it fail before we can complete the setup process. First, modify line 5 in examples/color.js to contain the Bluetooth device ID of your Sphero. Next launch color.js using node:
```
cd examples
node color.js
```
This should produce an error. To correct it, run:
```
sudo setcap cap_net_raw+eip $(eval readlink -f `which node`)
```
Now we should be ready to run roscore, our command node, and python scripts.

## Launching ROS, Sphero node, and demo rospy app:
In one terminal window:
```
roscore
```

In a second terminal window:
```
cd [SPHERE_GAMES_REPO_PATH]/ros_comms/
node sphero_command.js
```

In a third terminal window:
```
source ~/python_env/bin/activate
cd [SPHERE_GAMES_REPO_PATH]/ros_comms/
python rospy_demo.py
```
