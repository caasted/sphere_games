# proof_of_concept
## King of the Hill Proof of Concept

ROS/Gazebo simulation of sphere robots for reinforcement learning activities

Designed in Ubuntu 16.04 using ROS Kinetic and Gazebo 7.

## Installation
Make sure the following commands have been added to your .bashrc file or ran in the terminal used to launch rosrun:
```
source /opt/ros/kinetic/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:[INSERT_PATH_TO_REPO]/proof_of_concept/build
```

Copy all files in sphere_games/models/ to your ~/.gazebo/models/ directory

Create and configure a virtual environment for managing Python packages:
```
sudo apt install virtualenv python-opencv
mkdir python2_env
virtualenv python2_env --python=python2.7
source python2_env/bin/activate
pip install --upgrade pip
pip install numpy pyyaml rospkg catkin_pkg
```

## Build
In a terminal, 
```
cd [REPO_PATH]/proof_of_concept/
mkdir build
cd build
cmake ../
make
```

## Execution
In a terminal, run `roscore`

In a second terminal, 
```
cd [REPO_PATH]/proof_of_concept/
source /usr/share/gazebo/setup.sh
rosrun gazebo_ros gazebo proof_of_concept_arena.world
```

In a third terminal, run the sphere tracker python script:
```
source ~/python2_env/bin/activate
cd [repo_path]/proof_of_concept/
python sphere_tracker.py
```

In a fourth terminal, run the simple reinforcement learning agent script:
```
source ~/python2_env/bin/activate
cd [repo_path]/proof_of_concept/
python simple_rl_agent.py
```

The red sphere will learn to move towards the center of the frame, using Q-learning, while the other three spheres will move about randomly, occasionally knocking the red sphere out of its oscillations near the center. 

By 17 seconds into the video below, the red sphere has a strong tendency to stay near the center of the image.

[![8000 epochs of Q-learning](https://i9.ytimg.com/vi/6YAlH6CpblQ/hqdefault.jpg?sqp=CJSipNcF&rs=AOn4CLC3M6p1sq_V0l-I9Bdm9P8slTDhOA)](https://www.youtube.com/embed/6YAlH6CpblQ)

