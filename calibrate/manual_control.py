import sys
from getkey import getkey, keys

import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3

# Global variables
heading = 0
speed = 0

# Helper functions
def yaw_vel_to_twist(yaw, vel):
    twist_msg = Twist() 
    twist_msg.linear = Vector3(0, 0, 0) 
    twist_msg.angular.x = np.cos(yaw) * vel 
    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.z = 0 
    return twist_msg

# User inputs
def get_user_commands():
    global heading, speed
    reset_msg = False
    key = getkey()
    if key == 'q':
        reset_msg = True
    elif key == keys.UP:
        speed += 1
    elif key == keys.DOWN:
        speed -= 1
    elif key == keys.LEFT:
        heading += np.pi / 16
    elif key == keys.RIGHT:
        heading -= np.pi / 16

    if speed < 0:
        speed = 0
    elif speed >= 250:
        speed = 250

    if heading < 0:
        heading += 360
    heading = heading % 360

    twist_msg = yaw_vel_to_twist(heading, speed)

    # Reset Sphero internal heading
    if reset_msg:
        twist_msg.linear.z = -1

    return twist_msg, reset_msg

# Init function
def manual_control():
    global game_over
    # Setup ROS message handling
    rospy.init_node('blue_agent', anonymous=True)

    # Pass command line argument '1' for red, otherwise blue
    if len(sys.argv) > 1 and sys.argv[1] == '1':
        pub_cmd = rospy.Publisher('/red_sphero/twist_cmd', Twist, queue_size=1)
    else:
        pub_cmd = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)

    # Agent control loop
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        twist_msg, reset_msg = get_user_commands()
        pub_cmd.publish(twist_msg)
        if reset_msg != False:
            break
        rate.sleep()
    return

if __name__ == '__main__':
    try:
        manual_control()
    except rospy.ROSInterruptException:
        pass

