# ROS Comms
Exposes basic Sphero interface as a ROS node, allowing control via rospy.

## Installation
First, connect to your Sphero SPRK+ via Bluetooth. Then, install Node.JS, Sphero.JS, and rosnodejs:
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
Press `Control-C` to exit and then the following command to correct it:
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
```
Next, modify line 7 of "sphero_command.js" to match the Bluetooth device ID of your own Sphero SPRK+. Then run:
```
node sphero_command.js
```

In a third terminal window:
```
source ~/python_env/bin/activate
cd [SPHERE_GAMES_REPO_PATH]/ros_comms/
python rospy_demo.py
```
Your Sphero should now be randomly setting a new color and heading once per second. If the sphero_command terminal is throwing errors, try `control-c` and restarting it with `node sphero_command.js`.
