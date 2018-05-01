import numpy as np
import rospy

import std_msgs.msg
from geometry_msgs.msg import Point

red_yaw = 0.
red_vel = 0.
blue_yaw = 0.
blue_vel = 0.
green_yaw = 0.
green_vel = 0.
purple_yaw = 0.
purple_vel = 0.

center_x = 1920 / 2.
center_y = 1080 / 2.

red_Q_table = {}

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

    grid_x, grid_y, current_value = get_grid(sphere_center)

    if 'previous_value' in red_Q_table:
        previous_value = red_Q_table['previous_value']
        previous_grid = red_Q_table['previous_grid']
        previous_choice = red_Q_table['previous_choice']
        reward = current_value - previous_value
        red_Q_table[previous_grid][previous_choice] += reward

    if (np.random.random() > epoch / 2000. 
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
    print "Epoch: {}, Position Value: {}".format(epoch, current_value)

    red_yaw = yaw_choice
    red_vel = vel_choice
    return

def blue_sphere(sphere_center):
    global blue_yaw, blue_vel
    blue_yaw = np.random.choice(yaw_actions)
    blue_vel = np.random.choice(vel_actions)
    return

def green_sphere(sphere_center):
    global green_yaw, green_vel
    green_yaw = np.random.choice(yaw_actions)
    green_vel = np.random.choice(vel_actions)
    return

def purple_sphere(sphere_center):
    global purple_yaw, purple_vel
    purple_yaw = np.random.choice(yaw_actions)
    purple_vel = np.random.choice(vel_actions)
    return

def init_spheres():
    sub_red_center = rospy.Subscriber('/red_sphere/center', Point, red_sphere, queue_size=1)
    sub_blue_center = rospy.Subscriber('/blue_sphere/center', Point, blue_sphere, queue_size=1)
    sub_green_center = rospy.Subscriber('/green_sphere/center', Point, green_sphere, queue_size=1)
    sub_purple_center = rospy.Subscriber('/purple_sphere/center', Point, purple_sphere, queue_size=1)
    rospy.init_node('sphere_command', anonymous=True)

    pub_red_vel = rospy.Publisher('/red_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_red_yaw = rospy.Publisher('/red_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)
    
    pub_blue_vel = rospy.Publisher('/blue_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_blue_yaw = rospy.Publisher('/blue_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)
    
    pub_green_vel = rospy.Publisher('/green_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_green_yaw = rospy.Publisher('/green_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)
    
    pub_purple_vel = rospy.Publisher('/purple_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_purple_yaw = rospy.Publisher('/purple_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    global epoch
    rate = rospy.Rate(5) # Hz
    while not rospy.is_shutdown():
        pub_red_yaw.publish(red_yaw)
        pub_red_vel.publish(red_vel)
        pub_blue_yaw.publish(blue_yaw)
        pub_blue_vel.publish(blue_vel)
        pub_green_yaw.publish(green_yaw)
        pub_green_vel.publish(green_vel)
        pub_purple_yaw.publish(purple_yaw)
        pub_purple_vel.publish(purple_vel)
        rate.sleep()
        epoch += 1
    return

if __name__ == '__main__':
    try:
        init_spheres()
    except rospy.ROSInterruptException:
        pass

