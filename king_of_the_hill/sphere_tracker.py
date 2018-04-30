import numpy as np
import rospy
import cv2
from PIL import Image

import std_msgs.msg
from sensor_msgs.msg import CompressedImage

def receive_image(image_data):
    # print "Received image: {}".format(image_data.format)
    image_array = np.fromstring(image_data.data, np.uint8)
    cv2_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    cv2.imshow('cv2_image', cv2_image)
    cv2.waitKey(2)
    return

def randomly_move_spheres():
    pub_red_vel = rospy.Publisher('/red_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_red_yaw = rospy.Publisher('/red_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    pub_blue_vel = rospy.Publisher('/blue_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_blue_yaw = rospy.Publisher('/blue_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    pub_green_vel = rospy.Publisher('/green_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_green_yaw = rospy.Publisher('/green_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    pub_purple_vel = rospy.Publisher('/purple_sphere/vel_cmd', std_msgs.msg.Float32, queue_size=1)
    pub_purple_yaw = rospy.Publisher('/purple_sphere/yaw_cmd', std_msgs.msg.Float32, queue_size=1)

    sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, receive_image, queue_size=1)

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

