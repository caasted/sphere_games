import numpy as np
import rospy
import cv2

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

red_center = Point(0, 0, 0)
blue_center = Point(0, 0, 0)
green_center = Point(0, 0, 0)
purple_center = Point(0, 0, 0)

counter = 0

def receive_image(image_data):
    global red_center, blue_center, green_center, purple_center #, counter

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
    if len(red_contours) > 0:
        red_M = cv2.moments(max(red_contours, key=cv2.contourArea))
        red_center = Point(int(red_M['m10'] / red_M['m00']), int(red_M['m01'] / red_M['m00']), 0)

    blue_contours = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(blue_contours) > 0:
        blue_M = cv2.moments(max(blue_contours, key=cv2.contourArea))
        blue_center = Point(int(blue_M['m10'] / blue_M['m00']), int(blue_M['m01'] / blue_M['m00']), 0)

    green_contours = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(green_contours) > 0:
        green_M = cv2.moments(max(green_contours, key=cv2.contourArea))
        green_center = Point(int(green_M['m10'] / green_M['m00']), int(green_M['m01'] / green_M['m00']), 0)

    purple_contours = cv2.findContours(purple_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(purple_contours) > 0:
        purple_M = cv2.moments(max(purple_contours, key=cv2.contourArea))
        purple_center = Point(int(purple_M['m10'] / purple_M['m00']), int(purple_M['m01'] / purple_M['m00']), 0)

    print red_center
    print blue_center
    print green_center
    print purple_center
    # if counter % 20 == 0:
    #     cv2.imwrite('time_lapse/{0:08d}.png'.format(counter), cv2_image)
    counter += 1
    cv2.imshow('cv2_image', cv2_image)
    cv2.waitKey(2)
    return

def pub_sub_init():
    pub_red_center = rospy.Publisher('/red_sphere/center', Point, queue_size=1)
    pub_blue_center = rospy.Publisher('/blue_sphere/center', Point, queue_size=1)
    pub_green_center = rospy.Publisher('/green_sphere/center', Point, queue_size=1)
    pub_purple_center = rospy.Publisher('/purple_sphere/center', Point, queue_size=1)

    sub_image = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, receive_image, queue_size=1)

    rospy.init_node('sphere_tracker', anonymous=True)

    rate = rospy.Rate(5) # Hz
    while not rospy.is_shutdown():
        pub_red_center.publish(red_center)
        pub_blue_center.publish(blue_center)
        pub_green_center.publish(green_center)
        pub_purple_center.publish(purple_center)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

