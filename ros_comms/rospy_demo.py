import numpy as np
import rospy

import std_msgs.msg

def run_amok():
    rospy.init_node('sphere_command', anonymous=True)

    sphero_cmd = rospy.Publisher('/sphero/cmd', std_msgs.msg.String, queue_size=1)

    rate = rospy.Rate(1) # Hz
    while not rospy.is_shutdown():
        red = int(np.random.random() * 255)
        green = int(np.random.random() * 255)
        blue = int(np.random.random() * 255)
        color = '{0:02x}{1:02x}{2:02x}'.format(red, green, blue) 

        speed = 50
        heading = int(np.random.random() * 360)

        sphero_cmd.publish(str(speed) + ',' + str(heading) + ',' + color)
        rate.sleep()

    return

if __name__ == '__main__':
    try:
        run_amok()
    except rospy.ROSInterruptException:
        pass

