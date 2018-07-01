import os

import numpy as np
import rospy

import std_msgs.msg
from geometry_msgs.msg import Point, Twist, Vector3

red_twist = Twist()
blue_twist = Twist()

center_x = 1920 / 2.
center_y = 1080 / 2.

red_Q_table = {}
blue_Q_table = {}

yaw_actions = np.array(list(range(8))) * np.pi / 4
vel_actions = np.array(list(range(8))) * 2

epoch = 0

def get_heading_and_distance(sphere_center):
    delta_x = center_x - sphere_center.x
    delta_y = center_y - sphere_center.y
    heading = np.arctan2(delta_y, delta_x)
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    location_value = 1 - distance / 566.
    heading = int(4 * heading / np.pi)   # Convert to range(8)
    distance = int(8 * distance / 566.)  # Convert to range(8)
    return heading, distance, location_value

def Q_learning(sphere_center, Q_table):
    expectation = 0.

    heading, distance, current_value = get_heading_and_distance(sphere_center)

    if 'previous_value' in Q_table:
        previous_value = Q_table['previous_value']
        previous_grid = Q_table['previous_grid']
        previous_choice = Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.01 + current_value / 90.
        Q_value = Q_table[previous_grid][previous_choice]
        Q_table[previous_grid][previous_choice] = 0.9 * Q_value + 0.1 * reward

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
    if current_value > 0.8: # Edge of green hill
        Q_table['score'] += 1
    Q_table['previous_grid'] = (heading, distance)
    Q_table['previous_choice'] = (yaw_choice, vel_choice)
    # print "Epoch: {}, Position Value: {:.04}, Expected Reward: {:.04}".format(epoch, current_value, expectation)

    return yaw_choice, vel_choice

def yaw_vel_to_twist(yaw, vel):
    twist_msg = Twist()
    twist_msg.linear = Vector3(0, 0, 0)
    twist_msg.angular.x = np.cos(yaw) * vel
    twist_msg.angular.y = np.sin(yaw) * vel
    twist_msg.angular.z = 0
    return twist_msg

def red_sphere(sphere_center):
    global red_Q_table, red_twist
    red_yaw, red_vel = Q_learning(sphere_center, red_Q_table)
    red_twist = yaw_vel_to_twist(red_yaw, red_vel)
    return

def blue_sphere(sphere_center):
    global blue_Q_table, blue_twist
    blue_yaw, blue_vel = Q_learning(sphere_center, blue_Q_table)
    blue_twist = yaw_vel_to_twist(blue_yaw, blue_vel)
    return

def parse_dict(unformatted):
    formatted = {}
    for key in unformatted.item().keys():
        formatted[key] = unformatted.item().get(key)
    return formatted

def init_spheres():
    global epoch, red_Q_table, blue_Q_table, red_twist, blue_twist
    if os.path.isfile('red_agent.npy'):
        red_Q_table = parse_dict(np.load('red_agent.npy'))
        epoch = red_Q_table['epochs']
        print "Loaded red agent from file."
    else:
        red_Q_table['epochs'] = 0
        print "New agent started."
    if os.path.isfile('blue_agent.npy'):
        blue_Q_table = parse_dict(np.load('blue_agent.npy'))
        print "Loaded blue agent from file."

    # Always set scores back to zero
    red_Q_table['score'] = 0
    blue_Q_table['score'] = 0

    sub_red_center = rospy.Subscriber('/red_sphero/center', Point, red_sphere, queue_size=1)
    sub_blue_center = rospy.Subscriber('/blue_sphero/center', Point, blue_sphere, queue_size=1)
    rospy.init_node('sphere_command', anonymous=True)

    pub_red_twist = rospy.Publisher('/red_sphero/twist_cmd', Twist, queue_size=1)
    pub_blue_twist = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)

    rate = rospy.Rate(10) # Hz
    noise = 0.1
    while not rospy.is_shutdown():
        print "Epoch: {}, Scores - Red: {}, Blue: {}".format(epoch, 
            red_Q_table['score'], blue_Q_table['score'])

        pub_red_twist.publish(red_twist)
        pub_blue_twist.publish(blue_twist)

        rate.sleep()
        epoch += 1
        if epoch >= red_Q_table['epochs'] + 10000:
            red_Q_table['epochs'] = epoch
            print red_Q_table['epochs']
            np.save('red_agent.npy', red_Q_table)
            np.save('blue_agent.npy', blue_Q_table)
            print "Training complete. Agents saved."
            break
    return

if __name__ == '__main__':
    try:
        init_spheres()
    except rospy.ROSInterruptException:
        pass

