import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist, Vector3

# Global variables
blue_center = Point()
blue_flag = False
blue_base = Point()
red_base = Point()
blue_twist = Twist()
game_over = False

# Helper functions
def set_center(sphere_center):
    global blue_center
    blue_center = sphere_center
    return

def set_flag(flag_status):
    global blue_flag
    blue_flag = flag_status.data
    return

def set_game_over(game_state):
    global game_over
    game_over = game_state.data
    return

def set_blue_base(base):
    global blue_base
    blue_base = base
    return

def set_red_base(base):
    global red_base
    red_base = base
    return

def yaw_vel_to_twist(yaw, vel): 
    twist_msg = Twist() 
    twist_msg.linear = Vector3(0, 0, 0) 
    twist_msg.angular.x = np.cos(yaw) * vel 
    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.z = 0 
    return twist_msg

def get_heading_and_distance():
    global blue_center, blue_flag, blue_base, red_base
    if blue_flag != False: # Have flag, go home
        target_x = blue_base.x
        target_y = blue_base.y
    else: # Don't have flag, go to opponent's base
        target_x = red_base.x
        target_y = red_base.y
    delta_x = target_x - blue_center.x
    delta_y = target_y - blue_center.y
    print("[{}, {}]".format(delta_x, delta_y))
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance

# Agent function
def proportional_control():
    global blue_twist

    if blue_center != None:
        heading, distance = get_heading_and_distance()
        heading = -heading # Switch from camera to world coordinates
        # heading -= np.pi / 2 # Offset to calibrate heading
        speed = distance / 100.
    else:
        speed = 0
        heading = 0
    blue_twist = yaw_vel_to_twist(heading, speed)
    return

# Init function
def simple_agent():
    global game_over
    # Setup ROS message handling
    rospy.init_node('blue_agent', anonymous=True)

    pub_blue_cmd = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)
    sub_blue_center = rospy.Subscriber('/blue_sphero/center', Point, set_center, queue_size=1)
    sub_blue_flag = rospy.Subscriber('/blue_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)

    # Agent control loop
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        proportional_control()
        pub_blue_cmd.publish(blue_twist)
        if game_over != False:
            break
        rate.sleep()
    print("Game ended. No agent to save.")
    return

if __name__ == '__main__':
    try:
        simple_agent()
    except rospy.ROSInterruptException:
        pass

