import numpy as np
import rospy

import std_msgs.msg

def randomly_move_spheres():
    pub_red_vel = rospy.Publisher('/red_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_red_yaw = rospy.Publisher('/red_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    pub_blue_vel = rospy.Publisher('/blue_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_blue_yaw = rospy.Publisher('/blue_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    pub_green_vel = rospy.Publisher('/green_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_green_yaw = rospy.Publisher('/green_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    pub_purple_vel = rospy.Publisher('/purple_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_purple_yaw = rospy.Publisher('/purple_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    rospy.init_node('sphere_command', anonymous=True)
    rate = rospy.Rate(0.5) # Hz

    while not rospy.is_shutdown():
        pub_red_yaw.publish(2 * np.pi * np.random.random())
        pub_red_vel.publish(10 * np.random.random() + 5)
        pub_blue_yaw.publish(2 * np.pi * np.random.random())
        pub_blue_vel.publish(10 * np.random.random() + 5)
        pub_green_yaw.publish(2 * np.pi * np.random.random())
        pub_green_vel.publish(10 * np.random.random() + 5)
        pub_purple_yaw.publish(2 * np.pi * np.random.random())
        pub_purple_vel.publish(10 * np.random.random() + 5)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        randomly_move_spheres()
    except rospy.ROSInterruptException:
        pass

