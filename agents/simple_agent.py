import numpy as np
import rospy
from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import Point, Twist, Vector3
import host.utilities as util
import time

class simple_agent(object):

    def __init__(self, color):

        # Class Variables
        self.color = color
        self.center = Point(0,0,0)
        self.flag = False
        self.game_state = None
        self.game_over = None
        self.my_base = None
        self.their_base = None
        self.last_position = None
        self.start_time = time.time()

        # Control Parameters
        self.Kp = .1;
        self.Ki = .001;
        self.Kd = .1;

        self.MAX_SPEED = 30

        if(color == 'blue'):
            self.opponent = 'red'
        else:
            self.opponent = 'blue'

    def set_center(self, pt):
        self.center = pt

    def set_flag(self, flag):
        self.flag = flag

    def set_my_base(self, base):
        self.my_base = base

    def set_their_base(self, base):
        self.their_base = base

    def set_game_state(self, state):
        self.game_state = int(state.data)
        pass

    def set_game_over(self, state):
        self.game_over = state

    def setup_ros(self):

        rospy.init_node(str(self.color) + '_simple_agent', anonymous=True)

        self.pub_vel_cmd       = rospy.Publisher('/'+str(self.color) + '_sphero/twist_cmd', Twist, queue_size=1)

        self.sub_center        = rospy.Subscriber('/'+str(self.color) + '_sphero/center_mm', Point, self.set_center, queue_size=1)
        self.sub_flag          = rospy.Subscriber('/'+str(self.color) + '_sphero/flag', Bool, self.set_flag, queue_size=1)
        self.sub_base          = rospy.Subscriber('/'+str(self.color) + '_sphero/base_mm', Point, self.set_my_base, queue_size=1)
        self.sub_opponent_base = rospy.Subscriber('/'+str(self.opponent) + '_sphero/base_mm', Point, self.set_their_base, queue_size=1)
        self.sub_game_over     = rospy.Subscriber('/game_over', Bool, self.set_game_over, queue_size=1)
        self.sub_game_state    = rospy.Subscriber('/arena/game_state', Int16, self.set_game_state, queue_size=1)

    def end_early(self):
        if(self.game_state !=1):
            return True

    def return_false(self):
        return False

    def test_game(self):
        while not rospy.is_shutdown():
            if(not self.their_base is None and not self.my_base is None):
                break
        self.go_to_position(self.their_base, self.return_false, have_flag=False)
        self.go_to_position(self.my_base, self.return_false, have_flag=True)


    def play_game(self):

        while not rospy.is_shutdown():
            if(not self.their_base is None and not self.my_base is None and not self.game_state is None):
                break

        start_msg_shown = False
        game_start_msg_shown = False
        game_end_msg_shown = False

        rate = rospy.Rate(2)  # Hz

        while not rospy.is_shutdown():

            if(self.game_state == 0): # Waiting for game to start
                if(not start_msg_shown):
                    print("Waiting for game to start...")

                    start_msg_shown = True
                    game_start_msg_shown = False
                    game_end_msg_shown = False
                pass
            elif(self.game_state == 1): # Game Active
                if(not game_start_msg_shown):
                    print("Starting Game...")
                    start_msg_shown = False
                    game_start_msg_shown = True
                    game_end_msg_shown = False
                self.go_to_position(self.their_base, self.end_early)
                self.go_to_position(self.my_base, self.end_early)
            elif(self.game_state == 2): # Game Over
                if (not game_end_msg_shown):
                    print("Game Ended")
                    start_msg_shown = False
                    game_start_msg_shown = False
                    game_end_msg_shown = True
                pass

            rate.sleep()

    def go_to_position(self, target, monitor_function, have_flag = False, allowed_error = 20, dwell_time = 10):
        print("Target Location: " + str(target))

        rate = rospy.Rate(2)  # Hz

        time_center = time.time()

        at_goal = False

        accumulated_error = 0

        while not rospy.is_shutdown() and not monitor_function():
            linear_error = util.calculate_distance(self.center, target)
            err_heading = util.calculate_error_heading(self.center, target)

            if(linear_error is None or err_heading is None):
                rate.sleep()
                continue

            if(np.abs(linear_error) < allowed_error):
                accumulated_error = 0
                if(not at_goal):
                    print("Touched Goal")
                    t = Twist()
                    t.linear = Vector3(0, 0, 0)
                    t.angular = Vector3(0, 0, err_heading)
                    self.pub_vel_cmd.publish(t)
                    at_goal = True
                    time_center = time.time()
                    rate.sleep()
                    continue
                else:
                    secs = time.time() - time_center
                    if(secs > dwell_time):
                        print("Found goal")
                        return
                    else:
                        print("Close to goal")
                        rate.sleep()
                        continue
            else:
                at_goal = False
                accumulated_error += linear_error

            # Control Parameters
            vel = linear_error * self.Kp + accumulated_error*self.Ki

            if(vel > self.MAX_SPEED):
                vel = self.MAX_SPEED

            t = Twist()
            t.linear = Vector3(vel, 0, 0)
            t.angular = Vector3(0, 0, err_heading)
            self.pub_vel_cmd.publish(t)

            rospy.sleep(2)

            t.linear = Vector3(0, 0, 0)
            t.angular = Vector3(0, 0, err_heading)
            self.pub_vel_cmd.publish(t)
            rospy.sleep(5)

            rate.sleep()




if(__name__ == "__main__"):
    b = simple_agent('blue')

    b.setup_ros()

    #b.test_game()

    b.play_game()
