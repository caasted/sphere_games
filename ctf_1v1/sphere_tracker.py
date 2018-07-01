import numpy as np
import rospy
import cv2

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

red_center = Point(0, 0, 0)
blue_center = Point(0, 0, 0)

counter = 0

def receive_image(image_data):
    global red_center, blue_center, counter

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

    red_contours = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(red_contours) > 0:
        red_M = cv2.moments(max(red_contours, key=cv2.contourArea))
        red_center = Point(int(red_M['m10'] / red_M['m00']), int(red_M['m01'] / red_M['m00']), 0)

    blue_contours = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(blue_contours) > 0:
        blue_M = cv2.moments(max(blue_contours, key=cv2.contourArea))
        blue_center = Point(int(blue_M['m10'] / blue_M['m00']), int(blue_M['m01'] / blue_M['m00']), 0)

    # counter += 1
    # if counter % 5 == 0:
    #     cv2.imwrite('time_lapse/{0:08d}.png'.format(counter), cv2_image)
    # cv2.imshow('cv2_image', cv2_image)
    # cv2.waitKey(2)
    return

def pub_sub_init():
    pub_red_center = rospy.Publisher('/red_sphero/center', Point, queue_size=1)
    pub_blue_center = rospy.Publisher('/blue_sphero/center', Point, queue_size=1)

    sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, receive_image, queue_size=1)

    rospy.init_node('sphere_tracker', anonymous=True)

    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        pub_red_center.publish(red_center)
        pub_blue_center.publish(blue_center)
        print("Red: [{}, {}]".format(red_center.x, red_center.y))
        print("Blue: [{}, {}]".format(blue_center.x, blue_center.y))
        rate.sleep()
        
if __name__ == '__main__':
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

