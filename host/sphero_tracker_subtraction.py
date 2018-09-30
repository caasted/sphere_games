import time

import numpy as np
import rospy
import cv2
import sys

from std_msgs.msg import Bool, Int16
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

class sphero_tracker_subtraction():
    '''
    Updated Tracker for Spheros
    '''

    def __init__(self):
        # Variables used by game
        self.red_base = Point(390, 749, 0)
        self.blue_base = Point(862, 249, 0)

        self.red_center = Point(0, 0, 0)
        self.blue_center = Point(0, 0, 0)

        # Who has a flag
        self.red_flag = False
        self.blue_flag = False

        # Game State
        self.game_state = 0 # 0 = Waiting, 1 = Running, 2 = Finished
        self.time_elapsed = 0 # Seconds
        self.start = None
        self.TOTAL_ALLOWED_TIME = 300 # Seconds

        # Current Score
        self.red_score = 0
        self.blue_score = 0

        self.initialized = False

        self.ref_image = None

        self.latest_img = None

        self.masked_image = None

        self.arena_image = None

        self.blur_level = 25

        self.bridge = CvBridge()

        version = cv2.__version__.split('.')[0]
        print "OpenCV Version: " + version

    def update_time(self):
        self.time_elapsed = time.time() - self.start
        if(self.time_elapsed >= self.TOTAL_ALLOWED_TIME):
            self.game_state = 2
            self.pub_game_over.publish(True)

    def update_game_state(self):

        if(self.game_state == 0): # Waiting State
            self.time_elapsed = 0
            pass
        elif(self.game_state == 1): # Running
            self.update_scoring()
            self.update_time()
        elif(self.game_state == 2): # Complete
            pass
        else: # Invalid States
            pass

    def process_start(self, do_start):

        # Start Game if in waiting state
        if(self.game_state == 0 and do_start.data == True):
            self.game_state = 1
            self.start = time.time()


    def process_reset(self, do_reset):

        # Start Game if in waiting state
        if(do_reset.data == True):
            self.game_state = 0
            self.start = None
            self.time_elapsed = 0
            self.red_score = 0
            self.blue_score = 0

    def init_publishers(self):
        self.pub_red_center     = rospy.Publisher('/red_sphero/center', Point, queue_size=1)
        self.pub_blue_center    = rospy.Publisher('/blue_sphero/center', Point, queue_size=1)
        self.pub_red_base       = rospy.Publisher('/red_sphero/base', Point, queue_size=1)
        self.pub_blue_base      = rospy.Publisher('/blue_sphero/base', Point, queue_size=1)
        self.pub_red_flag       = rospy.Publisher('/red_sphero/flag', Bool, queue_size=1)
        self.pub_blue_flag      = rospy.Publisher('/blue_sphero/flag', Bool, queue_size=1)
        self.pub_red_score      = rospy.Publisher('/red_sphero/score', Int16, queue_size=1)
        self.pub_blue_score     = rospy.Publisher('/blue_sphero/score', Int16, queue_size=1)

        self.pub_game_over      = rospy.Publisher('/game_over', Bool, queue_size=1)

        self.pub_game_state     = rospy.Publisher('/arena/game_state', Int16, queue_size=1)
        self.pub_time_elapsed   = rospy.Publisher('/arena/time_elapsed', Int16, queue_size=1)
        self.pub_masked_image   = rospy.Publisher('/arena/masked_image', Image, queue_size=1)
        self.pub_arena_image    = rospy.Publisher('/arena/game_image', Image, queue_size=1)

        self.sub_image = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.process_frame, queue_size=1)

        # Game Controls
        self.sub_image = rospy.Subscriber('/arena/start_game', Bool, self.process_start, queue_size=1)
        self.sub_image = rospy.Subscriber('/arena/reset_game', Bool, self.process_reset, queue_size=1)

        rospy.init_node('sphere_tracker', anonymous=True)

    def setup_reference_image(self, img):
        '''
        Create Reference Image to subtract from
        :return:
        '''

        # Check to confirm field is empty

        if sys.version_info[0] == 3:
            response = str(input("Is Field Clear of Spheros? (If not clear it now and type 'n') (Y/N):")).lower()
        else:
            response = raw_input("Is Field Clear of Spheros? (If not clear it now and type 'n') (Y/N):").lower()

        if(not(response == "y" or response == "yes")):
            return

        kernel = np.ones((15, 15), np.float32) / 225
        smoothed = cv2.filter2D(img, -1, kernel)

        cv2_img_blur = cv2.GaussianBlur(smoothed, (15,15),0)

        grey = cv2.cvtColor(cv2_img_blur, cv2.COLOR_BGR2GRAY)

        self.ref_image = grey
        self.initialized = True
        print("Tracker Initialized")

    def get_spheros(self, cv2_image):

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

        blue_center = None
        red_center = None

        red_contours = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(red_contours) > 0:
            red_M = cv2.moments(max(red_contours, key=cv2.contourArea))
            red_center = Point(int(red_M['m10'] / red_M['m00']), int(red_M['m01'] / red_M['m00']), 0)

        blue_contours = cv2.findContours(blue_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(blue_contours) > 0:
            blue_M = cv2.moments(max(blue_contours, key=cv2.contourArea))
            blue_center = Point(int(blue_M['m10'] / blue_M['m00']), int(blue_M['m01'] / blue_M['m00']), 0)

        return {'blue':blue_center, 'red':red_center}

    def get_mask(self, cv2_img):
        '''
        Take in CV2 Image and return masked image
        :param cv2_img:
        :return:
        '''

        # Smooth and Filter
        kernel = np.ones((15, 15), np.float32) / 225
        smoothed = cv2.filter2D(cv2_img, -1, kernel)

        cv2_img_blur = cv2.GaussianBlur(smoothed, (15,15),0)

        grey = cv2.cvtColor(cv2_img_blur, cv2.COLOR_BGR2GRAY)

        img_diff = cv2.absdiff(grey, self.ref_image)

        ret, mask = cv2.threshold(img_diff, 10, 255, cv2.THRESH_BINARY)

        masked_img = cv2.bitwise_and(cv2_img, cv2_img, mask=mask)

        return masked_img

    def process_frame(self, image_data):
        '''
        Take a specific image and identify sphero locations
        :return:
        '''

        cv2_img = self.bridge.compressed_imgmsg_to_cv2(image_data, desired_encoding="passthrough")

        self.latest_img = cv2_img

        # Check if initialized, if not save image
        if(not self.initialized):
            self.setup_reference_image(cv2_img)
            return

        # Mask Field
        masked_img = self.get_mask(cv2_img)
        self.masked_image = self.bridge.cv2_to_imgmsg(masked_img, encoding="bgr8")

        # Do Processing
        spheros = self.get_spheros(masked_img)

        # Return Sphero locations
        if(not spheros['red'] is None):
            self.red_center = spheros['red']

        if (not spheros['blue'] is None):
            self.blue_center = spheros['blue']

    # Scoring logic
    def update_scoring(self):

        red_at_away = False
        red_at_home = False
        blue_at_away = False
        blue_at_home = False

        threshold = 100
        if self.red_flag != False:
            distance = np.sqrt((self.red_center.x - self.red_base.x) ** 2 +
                               (self.red_center.y - self.red_base.y) ** 2)
            if distance < threshold:
                red_at_home = True
        else:
            distance = np.sqrt((self.red_center.x - self.blue_base.x) ** 2 +
                               (self.red_center.y - self.blue_base.y) ** 2)
            if distance < threshold:
                red_at_away = True

        if self.blue_flag != False:
            distance = np.sqrt((self.blue_center.x - self.blue_base.x) ** 2 +
                               (self.blue_center.y - self.blue_base.y) ** 2)
            if distance < threshold:
                blue_at_home = True
        else:
            distance = np.sqrt((self.blue_center.x - self.red_base.x) ** 2 +
                               (self.blue_center.y - self.red_base.y) ** 2)
            if distance < threshold:
                blue_at_away = True

        if red_at_home and blue_at_home:
            self.red_score += 1
            self.blue_score += 1
            self.red_flag = False
            self.blue_flag = False
        elif red_at_home:
            self.red_score += 1
            self.red_flag = False
            self.blue_flag = False
        elif blue_at_home:
            self.blue_score += 1
            self.red_flag = False
            self.blue_flag = False
        else:
            if red_at_away:
                self.red_flag = True
            if blue_at_away:
                self.blue_flag = True
        return

    def update_arena(self, img):
        # Write some Text

        arena_img = img.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (10, 40)
        fontScale = 1
        fontColor = (255, 255, 255)
        lineType = 2

        cv2.putText(arena_img, 'LM Autonomy Hackathon',
                    bottomLeftCornerOfText,
                    font,
                    fontScale,
                    fontColor,
                    lineType)

        # Game State
        if(self.game_state==0):
            game_text = "Waiting"
        elif(self.game_state==1):
            game_text = "Running"
        elif (self.game_state == 2):
            game_text = "Game Over"
        else:
            game_text = "ERROR"

        cv2.putText(arena_img, 'Status: '+ game_text,
                    (10, 870),
                    font,
                    fontScale,
                    fontColor,
                    lineType)

        # Time Remaining
        cv2.putText(arena_img, 'Time Remaining: ' + str(self.TOTAL_ALLOWED_TIME - self.time_elapsed) + ' s',
                    (10, 910),
                    font,
                    fontScale,
                    fontColor,
                    lineType)


        # Position Information
        cv2.putText(arena_img, 'Red (' + str(self.red_center.x) + ', ' + str(self.red_center.y)+')',
                    (1050,40),
                    font,
                    fontScale,
                    (0,0,255),
                    lineType)


        cv2.putText(arena_img, 'Blue (' + str(self.blue_center.x) + ', ' + str(self.blue_center.y)+')',
                    (780,40),
                    font,
                    fontScale,
                    (255,0,0),
                    lineType)

        # Score Information
        cv2.putText(arena_img, 'Red Team: ' + str(self.red_score),
                    (1050,910),
                    font,
                    fontScale,
                    (0,0,255),
                    lineType)


        cv2.putText(arena_img, 'Blue Team: ' + str(self.blue_score),
                    (780,910),
                    font,
                    fontScale,
                    (255,0,0),
                    lineType)

        # Sphero Locations
        if(self.red_flag):
            thickness = -1
        else:
            thickness = 2

        cv2.circle(arena_img, (self.red_center.x, self.red_center.y), 10, (0, 0, 255), thickness=thickness)

        if(self.blue_flag):
            thickness = -1
        else:
            thickness = 2

        cv2.circle(arena_img, (self.blue_center.x, self.blue_center.y), 10, (255, 0, 0), thickness=thickness)

        # Base Locations
        if (not self.red_flag):
            thickness = -1
        else:
            thickness = 2

        cv2.circle(arena_img, (self.red_base.x, self.red_base.y), 10, (0, 0, 255), thickness=thickness)

        if (not self.blue_flag):
            thickness = -1
        else:
            thickness = 2

        cv2.circle(arena_img, (self.blue_base.x, self.blue_base.y), 10, (255, 0, 0), thickness=thickness)

        return arena_img


    def start_tracking(self):

        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():

            self.update_game_state()

            if(not self.latest_img is None):
                self.arena_image = self.update_arena(self.latest_img)
                arena_image = self.bridge.cv2_to_imgmsg(self.arena_image, encoding="bgr8")
                self.pub_arena_image.publish(arena_image)

            if(not self.masked_image is None):
                self.pub_masked_image.publish(self.masked_image)

            # Publish Centers
            self.pub_red_center.publish(self.red_center)
            self.pub_blue_center.publish(self.blue_center)

            self.pub_red_base.publish(self.red_base)
            self.pub_blue_base.publish(self.blue_base)


            # Publish Game State
            self.pub_game_state.publish(self.game_state)
            self.pub_time_elapsed.publish(self.time_elapsed)
            self.pub_red_flag.publish(self.red_flag)
            self.pub_blue_flag.publish(self.blue_flag)

            self.pub_red_score.publish(self.red_score)
            self.pub_blue_score.publish(self.blue_score)

            self.pub_game_over.publish(False)

            rate.sleep()

        pass



if(__name__ == "__main__"):
    t = sphero_tracker_subtraction()
    t.init_publishers()
    t.start_tracking()
