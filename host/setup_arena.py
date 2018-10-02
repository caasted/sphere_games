import rospy
from std_msgs.msg import Bool, Int16, String, ColorRGBA
from geometry_msgs.msg import Point, Twist, Vector3, PointStamped
import sys
import numpy as np

import host.utilities as util

class ArenaSetup(object):

    def __init__(self):
        # Subscriptions
        self.sub_arena_centers = {}

        # Publications
        self.pub_sphero_cmd_vel = {}
        self.pub_sphero_set_heading = {}
        self.pub_sphero_set_color = {}

        self.TIME_TO_MOVE = 3
        self.CAL_RES_ANGLE = 5 # Angle to turn sphero when determining heading
        self.TOLERANCE = 10 # degrees
        self.SETTLE_TIME = 10

        self.robot_list = [
            #{'name': 'red_node' , 'color': ColorRGBA(255,0,0,0)},
            {'name': 'blue_node', 'color': ColorRGBA(0, 0, 255, 0)},
        ]
        pass

    def set_arena_center(self, robot, current_point):
        self.arena_center[robot] = current_point

    def set_robot_center(self, robot, current_point):
        self.arena_center[robot] = current_point

    def setup_ros(self):
        rospy.init_node('arena_setup', anonymous=True)

        for robot in self.robot_list:
            self.setup_robot(robot['name'])

    def setup_robot(self, name):

        # Subsciptions
        self.sub_arena_centers[name] = rospy.Subscriber('/arena/' + name + '/center_mm', PointStamped,
                                                  self.set_arena_center, callback_args=name, queue_size=1)

        # Publications
        self.pub_sphero_cmd_vel[name] = rospy.Publisher('/'+name+'/cmd_vel', Twist, queue_size=1)
        self.pub_sphero_set_heading[name] = rospy.Publisher('/'+name+'/set_heading', Int16, queue_size=1)
        self.pub_sphero_set_color[name] = rospy.Publisher('/'+name+'/set_color', ColorRGBA, queue_size=1)



    def reset_heading(self):
        twist_msg = Twist()
        twist_msg.linear = Vector3(0, 0, -1)
        twist_msg.angular = Vector3(0, 0, 0)
        return twist_msg

    def yaw_vel_to_twist(self,yaw, vel):
        twist_msg = Twist()
        twist_msg.linear = Vector3(vel, 0, 0)
        twist_msg.angular.z = yaw
        return twist_msg

    def turn_sphero(self,color, angle=90):
        self.pub_sphero_cmd[color].publish(self.yaw_vel_to_twist(angle, self.MOVE_VEL))

        rospy.sleep(self.SETTLE_TIME)

        cmd = self.reset_heading()

        self.pub_sphero_cmd[color].publish(cmd)
        print("Reset Heading")

    def set_angle(self, color, angle):
        t = Twist()
        t.linear = Vector3(0, 0, 0)
        t.angular = Vector3(0, 0, angle)
        self.pub_sphero_cmd[color].publish(t)
        rospy.sleep(self.TIME_TO_MOVE)

    def roll(self, color, speed, heading, time = 5):
        t = Twist()
        t.linear = Vector3(speed, 0, 0)
        t.angular = Vector3(0, 0, heading)
        self.pub_sphero_cmd[color].publish(t)
        rospy.sleep(time)

        t = Twist()
        t.linear = Vector3(0, 0, 0)
        t.angular = Vector3(0, 0, 0)
        self.pub_sphero_cmd[color].publish(t)

    def calibrate_sphero(self, robot):
        print("Starting Calibration")
        self.set_angle(robot, 0)

        # Get Robot initial position
        start = self.arena_center[robot]

        # Move robot slightly
        self.roll(robot, 50, 0)

        # Get updated position
        end = self.arena_center[robot]

        # calculate heading difference
        error_deg = int(util.calculate_error_heading(start, end))

        # Set new heading
        self.pub_sphero_set_heading[robot].publish(error_deg)

    def stop_sphero(self,color):
        self.pub_sphero_cmd_vel[color].publish(self.yaw_vel_to_twist(0, 0))

    def info_messages(self):
        print("Arena Setup v0.2")
        print("Please Verify that the tracker is running, both spheros are in the arena and their host nodes are running")
        if sys.version_info[0] == 3:
            response = str(input("Ready? (Y/N):")).lower()
        else:
            response = raw_input("Ready? (Y/N):").lower()

        if(not(response == "y" or response == "yes")):
            exit(0)

    def start_setup(self):
        '''
        Setup
        :return:
        '''
        # Info Messages
        self.info_messages()

        # Verify ROS is "running" in the arena
        self.setup_ros()

        for robot in self.robot_list:
            self.calibrate_sphero(robot['name'])

        pass


if(__name__ == "__main__"):
    a = ArenaSetup()
    a.start_setup()