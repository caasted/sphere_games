# sphere_games
ROS/Gazebo simulation of sphere robots for reinforcement learning activities

## Installation
Make sure the following commands have been added to your .bashrc file or ran in the terminal used to launch gzserver:
'''source /opt/ros/kinetic/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:[INSERT_PATH_TO_REPO]/king_of_the_hill/build'''

Copy all files in models/ to your ~/.gazebo/models/ directory

## Build


## Execution
In a terminal, run '''roscore'''

In a second terminal, cd into [REPO_PATH]/king_of_the_hill/ and then run '''gzserver --verbose proof_of_concept_arena.world'''

In a third terminal, run '''gzclient'''

In a fourth terminal, issue rostopic pub commands or execute python scripts.

rostopic pub example:
'''rostopic pub /blue_sphere/vel_cmd std_msgs/Float32 -- 1.57'''

