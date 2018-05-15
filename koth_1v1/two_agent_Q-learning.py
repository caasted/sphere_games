import numpy as np
import rospy

import std_msgs.msg
from geometry_msgs.msg import Point

red_yaw = 0.
red_vel = 0.
blue_yaw = 0.
blue_vel = 0.

center_x = 1920 / 2.
center_y = 1080 / 2.

red_Q_table = {}
blue_Q_table = {}

yaw_actions = np.array(list(range(8))) * np.pi / 4
vel_actions = np.array(list(range(1, 9))) * 2

epoch = 0

def get_grid(sphere_center):
    delta_x = center_x - sphere_center.x
    delta_y = center_y - sphere_center.y
    location_value = 1 - np.sqrt(delta_x ** 2 + delta_y ** 2) / 566.
    delta_x = int(delta_x / 100.) + 4 # Convert to range(8)
    delta_y = int(delta_y / 100.) + 4 # Convert to range(8)
    return delta_x, delta_y, location_value

def red_sphere(sphere_center):
    global red_Q_table, red_yaw, red_vel
    expectation = 0.

    grid_x, grid_y, current_value = get_grid(sphere_center)

    if 'previous_value' in red_Q_table:
        previous_value = red_Q_table['previous_value']
        previous_grid = red_Q_table['previous_grid']
        previous_choice = red_Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.01 + current_value / 50.
        red_Q_table[previous_grid][previous_choice] += reward

    if (epoch < 1000. 
        or (grid_x, grid_y) not in red_Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = red_Q_table[(grid_x, grid_y)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = red_Q_table[(grid_x, grid_y)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0:
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (grid_x, grid_y) not in red_Q_table:
        red_Q_table[(grid_x, grid_y)] = {}
    if (yaw_choice, vel_choice) not in red_Q_table[(grid_x, grid_y)]:
        red_Q_table[(grid_x, grid_y)][(yaw_choice, vel_choice)] = 0.
    red_Q_table['previous_value'] = current_value
    red_Q_table['previous_grid'] = (grid_x, grid_y)
    red_Q_table['previous_choice'] = (yaw_choice, vel_choice)
    print "Epoch: {}, Position Value: {:.04}, Expected Reward: {:.04}".format(epoch, current_value, expectation)

    red_yaw = yaw_choice
    red_vel = vel_choice
    return

def blue_sphere(sphere_center):
    global blue_Q_table, blue_yaw, blue_vel
    expectation = 0.

    grid_x, grid_y, current_value = get_grid(sphere_center)

    if 'previous_value' in blue_Q_table:
        previous_value = blue_Q_table['previous_value']
        previous_grid = blue_Q_table['previous_grid']
        previous_choice = blue_Q_table['previous_choice']
        reward = (current_value - previous_value) - 0.01 + current_value / 50.
        blue_Q_table[previous_grid][previous_choice] += reward

    if (epoch < 1000. 
        or (grid_x, grid_y) not in blue_Q_table):
        yaw_choice = np.random.choice(yaw_actions)
        vel_choice = np.random.choice(vel_actions)
    else:
        options = blue_Q_table[(grid_x, grid_y)].keys()
        highest = options[0]
        highest_value = -1000
        for option in options:
            option_value = blue_Q_table[(grid_x, grid_y)][option]
            if option_value > highest_value:
                highest = option
                highest_value = option_value
        if highest_value > 0:
            yaw_choice, vel_choice = highest
            expectation = highest_value
        else:
            yaw_choice = np.random.choice(yaw_actions)
            vel_choice = np.random.choice(vel_actions)

    if (grid_x, grid_y) not in blue_Q_table:
        blue_Q_table[(grid_x, grid_y)] = {}
    if (yaw_choice, vel_choice) not in blue_Q_table[(grid_x, grid_y)]:
        blue_Q_table[(grid_x, grid_y)][(yaw_choice, vel_choice)] = 0.
    blue_Q_table['previous_value'] = current_value
    blue_Q_table['previous_grid'] = (grid_x, grid_y)
    blue_Q_table['previous_choice'] = (yaw_choice, vel_choice)
    print "Epoch: {}, Position Value: {:.04}, Expected Reward: {:.04}".format(epoch, current_value, expectation)

    blue_yaw = yaw_choice
    blue_vel = vel_choice
    return

def init_spheres():
    sub_red_center = rospy.Subscriber('/red_sphere/center', Point, red_sphere, queue_size=1)
    sub_blue_center = rospy.Subscriber('/blue_sphere/center', Point, blue_sphere, queue_size=1)
    rospy.init_node('sphere_command', anonymous=True)

    pub_red_vel = rospy.Publisher('/red_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_red_yaw = rospy.Publisher('/red_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)
    
    pub_blue_vel = rospy.Publisher('/blue_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_blue_yaw = rospy.Publisher('/blue_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)
    
    global epoch
    rate = rospy.Rate(5) # Hz
    while not rospy.is_shutdown():
        pub_red_yaw.publish(red_yaw)
        pub_red_vel.publish(red_vel)
        pub_blue_yaw.publish(blue_yaw)
        pub_blue_vel.publish(blue_vel)
        rate.sleep()
        epoch += 1
    return

if __name__ == '__main__':
    try:
        init_spheres()
    except rospy.ROSInterruptException:
        pass

