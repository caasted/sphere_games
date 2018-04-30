# sphere_games
ROS/Gazebo simulation of sphere robots for reinforcement learning activities

## Installation
Make sure the following commands have been added to your .bashrc file or ran in the terminal used to launch gzserver:
```
source /opt/ros/kinetic/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:[INSERT_PATH_TO_REPO]/king_of_the_hill/build
```

Copy all files in models/ to your ~/.gazebo/models/ directory

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
cd [REPO_PATH]/king_of_the_hill/
mkdir build
cd build
cmake ../
make
```

## Execution
In a terminal, run `roscore`

In a second terminal, 
```
cd [REPO_PATH]/king_of_the_hill/
source /usr/share/gazebo/setup.sh
rosrun gazebo_ros gazebo proof_of_concept_arena.world
```

In a third terminal, issue rostopic pub commands or execute python scripts.

rostopic pub example:
```
rostopic pub /blue_sphere/vel_cmd std_msgs/Float32 -- 1.57
```

rospy pub example:
```
source ~/python2_env/bin/activate
cd [repo_path]/king_of_the_hill/
python sphere_tracker.py
```

Running the sphere_tracker script also issues random angular velocity and yaw commands to all four of the spheres.

