import numpy as np
import rospy
import cv2

from sensor_msgs.msg import CompressedImage

def receive_image(image_data):
    # print "Received image: {}".format(image_data.format)
    image_array = np.fromstring(image_data.data, np.uint8)
    cv2_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    cv2.imshow('cv2_image', cv2_image)
    cv2.waitKey(2)
    return

def pub_sub_init():
    sub_image = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, receive_image, queue_size=1)

    rospy.init_node('sphere_tracker', anonymous=True)

    rate = rospy.Rate(30) # Hz
    while not rospy.is_shutdown():
        rate.sleep()
    return

if __name__ == '__main__':
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

