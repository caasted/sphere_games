import rospy
from std_msgs.msg import Bool, Int16, String
from geometry_msgs.msg import Point, Twist, Vector3
import sys
import numpy as np

class ArenaSetup(object):

    def __init__(self):
        self.sub_centers = {'blue':None, 'red':None}
        self.pub_sphero_cmd = {'blue':None, 'red':None}
        self.pub_sphero_cal_cmd = {'blue': None, 'red': None}
        self.pub_sphero_color_cmd = {'blue': None, 'red': None}
        self.center = {'blue':None, 'red':None}

        self.TIME_TO_MOVE = 3
        self.CAL_RES_ANGLE = 5 # Angle to turn sphero when determining heading
        self.TOLERANCE = 10 # degrees
        self.SETTLE_TIME = 10

        pass

    def set_red_center(self, current_point):
        self.center['red'] = current_point

    def set_blue_center(self, current_point):
        self.center['blue'] = current_point

    def setup_ros(self):
        self.sub_centers['red'] = rospy.Subscriber('/red_sphero/center_mm', Point, self.set_red_center, queue_size=1)
        self.sub_centers['blue'] = rospy.Subscriber('/blue_sphero/center_mm', Point, self.set_blue_center, queue_size=1)

        self.pub_sphero_cmd['red'] = rospy.Publisher('/red_sphero/twist_cmd', Twist, queue_size=1)
        self.pub_sphero_cmd['blue'] = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)

        self.pub_sphero_cal_cmd['red'] = rospy.Publisher('/red_sphero/calibrate_cmd', Bool, queue_size=1)
        self.pub_sphero_cal_cmd['blue'] = rospy.Publisher('/blue_sphero/calibrate_cmd', Bool, queue_size=1)

        self.pub_sphero_color_cmd['red'] = rospy.Publisher('/red_sphero/color_cmd', String, queue_size=1)
        self.pub_sphero_color_cmd['blue'] = rospy.Publisher('/blue_sphero/color_cmd', String, queue_size=1)

        rospy.init_node('arena_setup', anonymous=True)

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

    def calculate_distance(self, start, end):
        distance = np.sqrt((start.x - end.x) ** 2 +
                           (start.y - end.y) ** 2)

        return  distance

    def calculate_error_heading(self, start, end):

        x = end.x - start.x
        y = end.y - start.y

        theta = np.arctan2(y,x)

        angle = 90 - np.rad2deg(theta) # Rotate so heading is from true north

        if(angle > 180):
            angle = angle - 360
        elif(angle < -180):
            angle = angle + 360

        return angle

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

    def calibrate_sphero(self,color):
        print("Starting Calibration")
        self.set_angle(color, 0)

        # Turn off all spheros
        print("Turning off all sphero colors")
        self.pub_sphero_color_cmd['red'].publish('#000000')
        self.pub_sphero_color_cmd['blue'].publish('#000000')

        rospy.sleep(self.TIME_TO_MOVE)

        self.pub_sphero_cal_cmd[color].publish(True)

        rospy.sleep(self.TIME_TO_MOVE)

        angle_vals = {}

        # Rough Cal
        print("Doing Rough Heading")
        for angle in range(0, 360, 45):
            print("Moving to: " + str(angle) + " degrees heading")
            self.set_angle(color, angle)
            pos = self.center['blue']
            angle_vals[angle] = pos.y

        best_angle = min(angle_vals, key=angle_vals.get)

        self.set_angle(color, best_angle)

        # Fine Cal
        print("Doing Fine Heading")
        for angle in range(best_angle-20, best_angle+20, 2):
            print("Moving to: " + str(angle) + " degrees heading")
            self.set_angle(color, angle)
            pos = self.center['blue']
            angle_vals[angle] = pos.y

        best_angle = min(angle_vals, key=angle_vals.get)

        self.set_angle(color, best_angle)

        self.pub_sphero_cal_cmd[color].publish(False)

        # Turn on all spheros
        self.pub_sphero_color_cmd['red'].publish('#200000')
        self.pub_sphero_color_cmd['blue'].publish('#000020')


    def stop_sphero(self,color):
        self.pub_sphero_cmd[color].publish(self.yaw_vel_to_twist(0, 0))

    def info_messages(self):
        print("Arena Setup v0.1")
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

        # Calibrate Blue Ball
        self.calibrate_sphero('blue')

        # Calibrate Red Ball
        self.calibrate_sphero('red')

        pass


if(__name__ == "__main__"):
    a = ArenaSetup()
    a.start_setup()