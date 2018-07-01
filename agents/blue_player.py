import numpy as np
import rospy

import std_msgs.msg
from geometry_msgs.msg import Point, Twist, Vector3

blue_center = None

def blue_sphere(sphere_center):
    global blue_center
    blue_center = sphere_center
    return

def yaw_vel_to_twist(yaw, vel): 
    twist_msg = Twist() 
    twist_msg.linear = Vector3(0, 0, 0) 
    twist_msg.angular.x = np.cos(yaw) * vel 
    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.z = 0 
    return twist_msg

def goto_center():
    rospy.init_node('blue_sphere_command', anonymous=True)

    sphero_cmd = rospy.Publisher('/blue_sphero/twist_cmd', Twist, queue_size=1)

    sub_blue_center = rospy.Subscriber('/blue_sphero/center', Point, blue_sphere, queue_size=1)

    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        if blue_center != None:
            delta_x = 640 - blue_center.x
            delta_y = 480 - blue_center.y
            distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
            heading = int(180 * np.arctan2(delta_y, delta_x) / np.pi)
            heading += 45
            while heading > 360 or heading < 0:
                if heading < 0:
                    heading += 360
                if heading > 360:
                    heading -= 360
            speed = int(0.05 * distance) + 20
        else:
            speed = 0
            heading = 0

        sphero_cmd.publish(yaw_vel_to_twist(heading, speed))
        rate.sleep()

    return

if __name__ == '__main__':
    try:
        goto_center()
    except rospy.ROSInterruptException:
        pass

