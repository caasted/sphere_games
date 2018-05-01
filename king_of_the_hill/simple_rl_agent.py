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

def red_sphere(sphere_center):
    delta_x = center_x - sphere_center.x
    delta_y = center_y - sphere_center.y

    global red_yaw, red_vel
    red_yaw = np.arctan2(-delta_y, delta_x)
    red_vel = np.sqrt(delta_x ** 2 + delta_y ** 2) / 10.
    return

def blue_sphere(sphere_center):
    delta_x = center_x - sphere_center.x
    delta_y = center_y - sphere_center.y

    global blue_yaw, blue_vel
    blue_yaw = np.arctan2(-delta_y, delta_x)
    blue_vel = np.sqrt(delta_x ** 2 + delta_y ** 2) / 30.
    return

def green_sphere(sphere_center):
    delta_x = center_x - sphere_center.x
    delta_y = center_y - sphere_center.y

    global green_yaw, green_vel
    green_yaw = np.arctan2(-delta_y, delta_x)
    green_vel = np.sqrt(delta_x ** 2 + delta_y ** 2) / 40.
    return

def purple_sphere(sphere_center):
    delta_x = center_x - sphere_center.x
    delta_y = center_y - sphere_center.y

    global purple_yaw, purple_vel
    purple_yaw = np.arctan2(-delta_y, delta_x)
    purple_vel = np.sqrt(delta_x ** 2 + delta_y ** 2) / 50.
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

    rate = rospy.Rate(0.5) # Hz
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
    return

if __name__ == '__main__':
    try:
        init_spheres()
    except rospy.ROSInterruptException:
        pass

