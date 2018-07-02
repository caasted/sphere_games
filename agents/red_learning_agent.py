import os

import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point, Vector3

# Global variables
red_center = Point()
red_flag = False
red_base = Point()
blue_base = Point()
game_over = False

red_twist = Twist()
Q_table = {}
yaw_actions = np.array(list(range(8))) * np.pi / 4
vel_actions = np.array(list(range(10, 18)))

# Helper functions
def set_center(sphere_center):
    global red_center
    red_center = sphere_center
    return

def set_flag(flag_status):
    global red_flag
    red_flag = flag_status.data
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

def parse_dict(unformatted):
    formatted = {}
    for key in unformatted.item().keys():
        formatted[key] = unformatted.item().get(key)
    return formatted

def get_heading_and_distance():
    global red_center, red_flag, red_base, blue_base
    if red_flag != False: # Have flag, go home
        target_x = red_base.x
        target_y = red_base.y
    else: # Don't have flag, go to opponent's base
        target_x = blue_base.x
        target_y = blue_base.y
    delta_x = target_x - red_center.x
    delta_y = target_y - red_center.y
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance

# Agent function
def Q_learning():
    global Q_table, red_twist, yaw_actions, vel_actions
    expectation = 0.

    heading, distance = get_heading_and_distance()
    current_value = -distance / 1200. # Scale to [0, ~-1]
    heading = int(4 * heading / np.pi)   # Convert to range(8)
    distance = int(8 * distance / 1200.)  # Convert to range(8)

    if 'previous_value' in Q_table:
        previous_value = Q_table['previous_value']
        previous_grid = Q_table['previous_grid']
        previous_choice = Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.001
        Q_value = Q_table[previous_grid][previous_choice]
        Q_table[previous_grid][previous_choice] += reward

    if ((heading, distance) not in Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = Q_table[(heading, distance)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = Q_table[(heading, distance)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0:
            print(highest_value)
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (heading, distance) not in Q_table:
        Q_table[(heading, distance)] = {}
    if (yaw_choice, vel_choice) not in Q_table[(heading, distance)]:
        Q_table[(heading, distance)][(yaw_choice, vel_choice)] = 0.
    Q_table['previous_value'] = current_value
    Q_table['previous_grid'] = (heading, distance)
    Q_table['previous_choice'] = (yaw_choice, vel_choice)

    print("Yaw: {}, Vel: {}, Value: {}".format(yaw_choice, vel_choice, 
        current_value))
    yaw_choice = -yaw_choice # Switch from camera to world coordinates
    # yaw_choice += np.pi / 2 # Offset to calibrate heading
    red_twist = yaw_vel_to_twist(yaw_choice, vel_choice)
    return

# Init function
def learning_agent():
    # Load any existing agent
    global Q_table
    if os.path.isfile('red_agent.npy'):
        Q_table = parse_dict(np.load('red_agent.npy'))
        print "Loaded red agent from file."
    else:
        print "New agent started."

    # Setup ROS message handling
    rospy.init_node('red_agent', anonymous=True)

    pub_red_cmd = rospy.Publisher('/red_sphero/twist_cmd', Twist, queue_size=1)
    sub_red_center = rospy.Subscriber('/red_sphero/center', Point, set_center, queue_size=1)
    sub_red_flag = rospy.Subscriber('/red_sphero/flag', Bool, set_flag, queue_size=1)
    sub_blue_base = rospy.Subscriber('/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_red_base = rospy.Subscriber('/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)

    # Agent control loop
    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        Q_learning()
        pub_red_cmd.publish(red_twist)
        if game_over != False:
            break
        rate.sleep()

    np.save('red_agent.npy', Q_table)
    print "Game ended. Agent saved."
    return

if __name__ == '__main__':
    try:
        learning_agent()
    except rospy.ROSInterruptException:
        pass

