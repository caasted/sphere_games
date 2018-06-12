import numpy as np
import rospy

import std_msgs.msg
from geometry_msgs.msg import Point

blue_center = None

def blue_sphere(sphere_center):
    global blue_center
    blue_center = sphere_center
    return

def goto_center():
    rospy.init_node('blue_sphere_command', anonymous=True)

    sphero_cmd = rospy.Publisher('/blue_sphero/cmd', std_msgs.msg.String, queue_size=1)

    sub_blue_center = rospy.Subscriber('/blue_sphero/center', Point, blue_sphere, queue_size=1)

    rate = rospy.Rate(5) # Hz
    while not rospy.is_shutdown():
        if blue_center != None:
            delta_x = 640 - blue_center.x
            delta_y = 480 - blue_center.y
            distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
            heading = int(180 * np.arctan2(delta_y, delta_x) / np.pi)
            heading += 35
            while heading > 360 or heading < 0:
                if heading < 0:
                    heading += 360
                if heading > 360:
                    heading -= 360
            speed = int(0.05 * distance) + 20
        else:
            speed = 0
            heading = 0

        sphero_cmd.publish(str(speed) + ',' + str(heading))
        rate.sleep()

    return

if __name__ == '__main__':
    try:
        goto_center()
    except rospy.ROSInterruptException:
        pass

