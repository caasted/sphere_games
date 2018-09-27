import time

import numpy as np
import rospy
import cv2
import sys
import constants
import utilities

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
        self.base = {'red':Point(390, 749, 0), 'blue': Point(862, 249, 0)}
        self.center = {'red':Point(0, 0, 0), 'blue':Point(0, 0, 0)}

        self.red_center_mm = Point(0, 0, 0)
        self.blue_center_mm = Point(0, 0, 0)

        red_x = (self.base['red'].x - constants.ORIGIN_PIXELS.x) * constants.COVERT_PIXEL2MM
        red_y = ((constants.PICTURE_SIZE[1] - self.base[
            'red'].y) - constants.ORIGIN_PIXELS.y) * constants.COVERT_PIXEL2MM
        red_z = (self.base['red'].z - constants.ORIGIN_PIXELS.z) * constants.COVERT_PIXEL2MM

        blue_x = (self.base['blue'].x - constants.ORIGIN_PIXELS.x) * constants.COVERT_PIXEL2MM
        blue_y = ((constants.PICTURE_SIZE[1] - self.base[
            'blue'].y) - constants.ORIGIN_PIXELS.y) * constants.COVERT_PIXEL2MM
        blue_z = (self.base['blue'].z - constants.ORIGIN_PIXELS.z) * constants.COVERT_PIXEL2MM

        self.red_base_mm = Point(red_x, red_y, red_z)

        self.blue_base_mm = Point(blue_x, blue_y, blue_z)

        # Who has a flag
        self.flag = {'red':False, 'blue' : False}

        # Game State
        self.game_state = 0 # 0 = Waiting, 1 = Running, 2 = Finished
        self.time_elapsed = 0 # Seconds
        self.start = None

        # Current Score
        self.score = {'red':0, 'blue':0}

        self.initialized = False

        # Available Images
        self.ref_image = None
        self.latest_img = None
        self.masked_image = None
        self.arena_image = None
        self.diff_image = None

        self.blur_level = 25

        self.bridge = CvBridge()

        version = cv2.__version__.split('.')[0]
        print "OpenCV Version: " + version

    def convert_pixels_mm(self, pt_pixels):
        x = pt_pixels.x - constants.ORIGIN_PIXELS.x
        y = -(pt_pixels.y - constants.ORIGIN_PIXELS.y) # flip so negative is down

        # Scale
        x_mm = x * constants.COVERT_PIXEL2MM
        y_mm = y * constants.COVERT_PIXEL2MM

        return Point(x,y,0)

    def update_time(self):
        self.time_elapsed = time.time() - self.start
        if(self.time_elapsed >= constants.TOTAL_ALLOWED_TIME):
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
            self.score['red'] = 0
            self.score['blue'] = 0

    def init_publishers(self):
        self.pub_red_center     = rospy.Publisher('/red_sphero/center', Point, queue_size=1)
        self.pub_blue_center    = rospy.Publisher('/blue_sphero/center', Point, queue_size=1)
        self.pub_red_center_mm  = rospy.Publisher('/red_sphero/center_mm', Point, queue_size=1)
        self.pub_blue_center_mm = rospy.Publisher('/blue_sphero/center_mm', Point, queue_size=1)
        self.pub_red_base       = rospy.Publisher('/red_sphero/base', Point, queue_size=1)
        self.pub_blue_base      = rospy.Publisher('/blue_sphero/base', Point, queue_size=1)
        self.pub_red_base_mm    = rospy.Publisher('/red_sphero/base_mm', Point, queue_size=1)
        self.pub_blue_base_mm   = rospy.Publisher('/blue_sphero/base_mm', Point, queue_size=1)
        self.pub_red_flag       = rospy.Publisher('/red_sphero/flag', Bool, queue_size=1)
        self.pub_blue_flag      = rospy.Publisher('/blue_sphero/flag', Bool, queue_size=1)
        self.pub_red_score      = rospy.Publisher('/red_sphero/score', Int16, queue_size=1)
        self.pub_blue_score     = rospy.Publisher('/blue_sphero/score', Int16, queue_size=1)

        self.pub_game_over      = rospy.Publisher('/game_over', Bool, queue_size=1)

        self.pub_game_state     = rospy.Publisher('/arena/game_state', Int16, queue_size=1)
        self.pub_time_elapsed   = rospy.Publisher('/arena/time_elapsed', Int16, queue_size=1)
        self.pub_masked_image   = rospy.Publisher('/arena/masked_image', Image, queue_size=1)
        self.pub_blue_mask      = rospy.Publisher('/arena/blue_mask', Image, queue_size=1)
        self.pub_red_mask       = rospy.Publisher('/arena/red_mask', Image, queue_size=1)
        self.pub_arena_image    = rospy.Publisher('/arena/game_image', Image, queue_size=1)
        self.pub_diff_image     = rospy.Publisher('/arena/diff_image', Image, queue_size=1)

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

    def find_contours(self, mask):
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        center = None

        if len(contours) > 0:
            moments = cv2.moments(max(contours, key=cv2.contourArea))
            center = Point(int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00']), 0)

        return center

    def find_circles(self, mask):
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20,
                                   param1 = 50, param2 = 10, minRadius = 0, maxRadius = 0)

        if(circles is None):
            return None

        for (x,y,r) in circles[0,:]:
            return Point(x,y,0)

        return None

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

        self.pub_blue_mask.publish(self.bridge.cv2_to_imgmsg(blue_mask, encoding="mono8"))
        self.pub_red_mask.publish(self.bridge.cv2_to_imgmsg(red_mask, encoding="mono8"))

        #blue_center = self.find_contours(blue_mask)
        #red_center = self.find_contours(red_mask)

        blue_center = self.find_circles(blue_mask)
        red_center = self.find_circles(red_mask)

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
        self.diff_image = self.bridge.cv2_to_imgmsg(img_diff, encoding="mono8")

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
            self.center['red'] = spheros['red']
            self.red_center_mm = self.convert_pixels_mm(spheros['red'])

        if (not spheros['blue'] is None):
            self.center['blue'] = spheros['blue']
            self.blue_center_mm = self.convert_pixels_mm(spheros['blue'])

    # Scoring logic
    def update_scoring(self):

        red_at_away = False
        red_at_home = False
        blue_at_away = False
        blue_at_home = False

        threshold = 100
        if self.flag['red'] != False:
            distance = np.sqrt((self.center['red'].x - self.base['red'].x) ** 2 +
                               (self.center['red'].y - self.base['red'].y) ** 2)
            if distance < threshold:
                red_at_home = True
        else:
            distance = np.sqrt((self.center['red'].x - self.base['blue'].x) ** 2 +
                               (self.center['red'].y - self.base['blue'].y) ** 2)
            if distance < threshold:
                red_at_away = True

        if self.flag['blue'] != False:
            distance = np.sqrt((self.center['blue'].x - self.base['blue'].x) ** 2 +
                               (self.center['blue'].y - self.base['blue'].y) ** 2)
            if distance < threshold:
                blue_at_home = True
        else:
            distance = np.sqrt((self.center['blue'].x - self.base['red'].x) ** 2 +
                               (self.center['blue'].y - self.base['red'].y) ** 2)
            if distance < threshold:
                blue_at_away = True

        if red_at_home and blue_at_home:
            self.score['red'] += 1
            self.score['blue'] += 1
            self.flag['red'] = False
            self.flag['blue'] = False
        elif red_at_home:
            self.score['red'] += 1
            self.flag['red'] = False
            self.flag['blue'] = False
        elif blue_at_home:
            self.score['blue'] += 1
            self.flag['red'] = False
            self.flag['blue'] = False
        else:
            if red_at_away:
                self.flag['red'] = True
            if blue_at_away:
                self.flag['blue'] = True
        return

    def start_tracking(self):

        rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():

            self.update_game_state()

            if(not self.latest_img is None):
                #self.arena_image = self.update_arena(self.latest_img)
                self.arena_image = utilities.update_arena(self.game_state, self.time_elapsed,
                                                          self.score, self.center, self.base,
                                                          self.flag, self.latest_img)
                arena_image = self.bridge.cv2_to_imgmsg(self.arena_image, encoding="bgr8")
                self.pub_arena_image.publish(arena_image)

            if(not self.masked_image is None):
                self.pub_masked_image.publish(self.masked_image)

            if (not self.diff_image is None):
                self.pub_diff_image.publish(self.diff_image)

            # Publish Centers
            self.pub_red_center.publish(self.center['red'])
            self.pub_blue_center.publish(self.center['blue'])
            self.pub_red_center_mm.publish(self.red_center_mm)
            self.pub_blue_center_mm.publish(self.blue_center_mm)

            self.pub_red_base.publish(self.base['red'])
            self.pub_blue_base.publish(self.base['blue'])

            self.pub_red_base_mm.publish(self.red_base_mm)
            self.pub_blue_base_mm.publish(self.blue_base_mm)

            # Publish Game State
            self.pub_game_state.publish(self.game_state)
            self.pub_time_elapsed.publish(self.time_elapsed)
            self.pub_red_flag.publish(self.flag['red'])
            self.pub_blue_flag.publish(self.flag['blue'])

            self.pub_red_score.publish(self.score['red'])
            self.pub_blue_score.publish(self.score['blue'])

            self.pub_game_over.publish(False)

            rate.sleep()

        pass



if(__name__ == "__main__"):
    t = sphero_tracker_subtraction()
    t.init_publishers()
    t.start_tracking()