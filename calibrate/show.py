#! /usr/bin/env python

from __future__ import print_function

import numpy as np
import cv2

import sys
import rospy

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# Topics for image
TOPIC = { 'g': ['/arena/game_image',Image],
          'c': ['/raspicam_node/image/compressed',CompressedImage],
          'm': ['/arena/masked_image',Image],
          'd': ['/arena/diff_image',Image],
          'b': ['/arena/blue_mask',Image],
          'r': ['/arena/red_mask',Image],
}

WINDOW_NAME = 'DISPLAY'

bridge = None
which = 'g'

def show_image(image):
    global bridge, which
    
    if TOPIC[which][1] is CompressedImage:
        image_array = np.fromstring(image.data, np.uint8)
        cvimage = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    else: #Image
        cvimage = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

    cv2.imshow(WINDOW_NAME, cvimage)
#end

if __name__ == '__main__':
    if len(sys.argv)>1:
        which = sys.argv[1][0]
        if which not in TOPIC:
            print("Unrecognized topic code '%s', try %s" % (which,TOPIC.keys()))
            sys.exit("Invalid topic code")
    else:
       print("Please specify the code of the image to view:")
       for c in TOPIC: print("    Code '%s': topic %s" % (c,TOPIC[c][0]))
       sys.exit("Missing topic code")

    print("Displaying %s from %s" % (TOPIC[which][1],TOPIC[which][0]))
    try:
        bridge = CvBridge()
        cv2.namedWindow(WINDOW_NAME, flags=cv2.WINDOW_NORMAL) #or WINDOW_AUTOSIZE
        #autosize starts based on image and prevents resize
        
        # Setup ROS message handling
        rospy.init_node('show', anonymous=True)
        rospy.Subscriber(TOPIC[which][0], TOPIC[which][1],
                         show_image, queue_size=1)

        cv2.waitKey(0)          # Any key
    except rospy.ROSInterruptException:
        pass

    cv2.destroyWindow(WINDOW_NAME)
#end
