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

    hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

    red_lower_1 = np.array([0, 50, 50])
    red_upper_1 = np.array([int(1.0 * 180. / 6.), 255, 255])
    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    red_mask_1 = cv2.erode(red_mask_1, None, iterations=2)
    red_mask_1 = cv2.dilate(red_mask_1, None, iterations=2)

    red_lower_2 = np.array([int(5.5 * 180. / 6.), 50, 50])
    red_upper_2 = np.array([255, 255, 255])
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    red_mask_2 = cv2.erode(red_mask_2, None, iterations=2)
    red_mask_2 = cv2.dilate(red_mask_2, None, iterations=2)

    red_mask = red_mask_1 + red_mask_2

    blue_lower = np.array([int(3.0 * 180. / 6.), 50, 50])
    blue_upper = np.array([int(4.5 * 180. / 6.), 255, 255])
    blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    blue_mask = cv2.erode(blue_mask, None, iterations=2)
    blue_mask = cv2.dilate(blue_mask, None, iterations=2)

    green_lower = np.array([int(1.0 * 180. / 6.), 50, 50])
    green_upper = np.array([int(3.0 * 180. / 6.), 255, 255])
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    green_mask = cv2.erode(green_mask, None, iterations=2)
    green_mask = cv2.dilate(green_mask, None, iterations=2)

    purple_lower = np.array([int(4.5 * 180. / 6.), 50, 50])
    purple_upper = np.array([int(5.5 * 180. / 6.), 255, 255])
    purple_mask = cv2.inRange(hsv, purple_lower, purple_upper)
    purple_mask = cv2.erode(purple_mask, None, iterations=2)
    purple_mask = cv2.dilate(purple_mask, None, iterations=2)

    red_contours = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    red_center = None
    if len(red_contours) > 0:
        red_M = cv2.moments(max(red_contours, key=cv2.contourArea))
        red_center = (int(red_M['m10'] / red_M['m00']), int(red_M['m01'] / red_M['m00']))

    blue_contours = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    blue_center = None
    if len(blue_contours) > 0:
        blue_M = cv2.moments(max(blue_contours, key=cv2.contourArea))
        blue_center = (int(blue_M['m10'] / blue_M['m00']), int(blue_M['m01'] / blue_M['m00']))

    green_contours = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    green_center = None
    if len(green_contours) > 0:
        green_M = cv2.moments(max(green_contours, key=cv2.contourArea))
        green_center = (int(green_M['m10'] / green_M['m00']), int(green_M['m01'] / green_M['m00']))

    purple_contours = cv2.findContours(purple_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    purple_center = None
    if len(purple_contours) > 0:
        purple_M = cv2.moments(max(purple_contours, key=cv2.contourArea))
        purple_center = (int(purple_M['m10'] / purple_M['m00']), int(purple_M['m01'] / purple_M['m00']))

    print red_center, blue_center, green_center, purple_center
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

