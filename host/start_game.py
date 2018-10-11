import rospy
from std_msgs.msg import Bool

pub_start = rospy.Publisher('/arena/start_game', Bool, queue_size=1)

rospy.init_node('arena_start', anonymous=True)

pub_start.publish(True)