# ROS Comms
Exposes basic Sphero interface as a ROS node, allowing control via rospy.

## Installation
Connect to your Sphero SPRK+ via Bluetooth. 
Modify line 7 of "sphero_command.js" to match the Bluetooth device ID of your own Sphero SPRK+.

Install Node.JS, Sphero.JS, and rosnodejs:
```
cd ~/
curl -sL https://deb.nodesource.com/setup-8.x | sudo -E bash -
sudo apt install nodejs
git clone https://github.com/orbotix/sphero.js
cd sphero.js
npm install noble
sudo npm install -g
sudo npm install rosnodejs -g
sudo setcap cap_net_raw+eip $(eval readlink -f 'which node')
```

## Launching ROS, Sphero node, and demo rospy app:
In one terminal window:
'''
roscore
'''

In a second terminal window:
'''
cd [SPHERE_GAMES_REPO_PATH]/ros_comms/
node sphero_command.js
```

In a third terminal window:
```
source ~/python_env/bin/activate
cd [SPHERE_GAMES_REPO_PATH]/ros_comms/
python rospy_demo.py
```

