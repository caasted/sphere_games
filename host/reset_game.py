import rospy
from std_msgs.msg import Bool

pub_reset = rospy.Publisher('/arena/reset_game', Bool, queue_size=1)
pub_blue_flag = rospy.Publisher('/blue_sphero/flag', Bool, queue_size=1)
pub_red_flag = rospy.Publisher('/blue_sphero/flag', Bool, queue_size=1)

rospy.init_node('arena_reset', anonymous=True)

pub_reset.publish(True)
pub_blue_flag.publish(False)
pub_red_flag.publish(False)