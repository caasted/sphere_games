import time

import numpy as np
import rospy
import cv2

from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point

# Capture the Flag base configuration
print("Starting Capture the Flag")
red_base = Point(1022, 36, 0)
blue_base = Point(275, 650, 0)

red_center = Point(0, 0, 0)
blue_center = Point(0, 0, 0)
red_flag = False
blue_flag = False
red_score = 0
blue_score = 0

counter = 0

start = time.time()

def receive_image(image_data):
    global red_center, blue_center, counter

    image_array = np.fromstring(image_data.data, np.uint8)
    cv2_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

    # Mask by hue and find center
    hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

    red_lower_1 = np.array([0, 50, 50])
    red_upper_1 = np.array([10, 255, 255])
    red_mask_1 = cv2.inRange(hsv, red_lower_1, red_upper_1)

    red_lower_2 = np.array([160, 50, 50])
    red_upper_2 = np.array([180, 255, 255])
    red_mask_2 = cv2.inRange(hsv, red_lower_2, red_upper_2)

    red_mask = red_mask_1 + red_mask_2
    red_mask = cv2.erode(red_mask, None, iterations=2)
    red_mask = cv2.dilate(red_mask, None, iterations=2)

    blue_lower = np.array([110, 50, 50])
    blue_upper = np.array([120, 255, 255])
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

    # cv2.imshow('red mask', red_mask)
    # cv2.imshow('blue mask', blue_mask)
    cv2.imshow('cv2_image', cv2_image)
    cv2.waitKey(1)

    # counter += 1
    # if counter % 5 == 0:
    #     cv2.imwrite('time_lapse/{0:08d}.png'.format(counter), cv2_image)
    # cv2.imshow('cv2_image', cv2_image)
    # cv2.waitKey(2)

    return

# Scoring logic
def host():
    global red_center, blue_center, red_base, blue_base
    global red_flag, blue_flag, red_score, blue_score
    red_at_away = False
    red_at_home = False
    blue_at_away = False
    blue_at_home = False

    threshold = 100
    if red_flag != False:
        distance = np.sqrt((red_center.x - red_base.x) ** 2 +
                           (red_center.y - red_base.y) ** 2)
        if distance < threshold:
            red_at_home = True
    else:
        distance = np.sqrt((red_center.x - blue_base.x) ** 2 +
                           (red_center.y - blue_base.y) ** 2)
        if distance < threshold:
            red_at_away = True

    if blue_flag != False:
        distance = np.sqrt((blue_center.x - blue_base.x) ** 2 +
                           (blue_center.y - blue_base.y) ** 2)
        if distance < threshold:
            blue_at_home = True
    else:
        distance = np.sqrt((blue_center.x - red_base.x) ** 2 +
                           (blue_center.y - red_base.y) ** 2)
        if distance < threshold:
            blue_at_away = True

    if red_at_home and blue_at_home:
        red_score += 1
        blue_score += 1
        red_flag = False
        blue_flag = False
    elif red_at_home:
        red_score += 1
        red_flag = False
        blue_flag = False
    elif blue_at_home:
        blue_score += 1
        red_flag = False
        blue_flag = False
    else:
        if red_at_away:
            red_flag = True
        if blue_at_away:
            blue_flag = True
    return

def pub_sub_init():
    global red_center, blue_center, red_flag, blue_flag, red_score, blue_score

    pub_red_center = rospy.Publisher('/red_sphero/center', Point, queue_size=1)
    pub_blue_center = rospy.Publisher('/blue_sphero/center', Point, queue_size=1)
    pub_red_base = rospy.Publisher('/red_sphero/base', Point, queue_size=1)
    pub_blue_base = rospy.Publisher('/blue_sphero/base', Point, queue_size=1)
    pub_red_flag = rospy.Publisher('/red_sphero/flag', Bool, queue_size=1)
    pub_blue_flag = rospy.Publisher('/blue_sphero/flag', Bool, queue_size=1)

    pub_game_over = rospy.Publisher('/game_over', Bool, queue_size=1)

    sub_image = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, receive_image, queue_size=1)

    rospy.init_node('sphere_tracker', anonymous=True)

    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown():
        host()

        pub_red_center.publish(red_center)
        pub_blue_center.publish(blue_center)
        pub_red_base.publish(red_base)
        pub_blue_base.publish(blue_base)
        pub_red_flag.publish(red_flag)
        pub_blue_flag.publish(blue_flag)
        pub_game_over.publish(False)

        print("Time: {} / 300".format(time.time() - start))
        print("Red: [{}, {}], [{}, {}]".format(red_center.x, red_center.y,
            red_flag, red_score))
        print("Blue: [{}, {}], [{}, {}]".format(blue_center.x, blue_center.y,
            blue_flag, blue_score))

        if time.time() - start > 300:
            pub_game_over.publish(True)
            break

        rate.sleep()

    cv2.destroyAllWindows()

    if blue_score > red_score:
        print("Winner: Blue Team")
    elif blue_score < red_score:
        print("Winner: Red Team")
    else:
        print("Draw!")
    print("Final Score - Red: {}, Blue: {}".format(red_score, blue_score))
    return

if __name__ == '__main__':
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

