import rospy
from picamera import PiCamera
from time import sleep

from sensor_msgs.msg import Image

if __name__ == "__main__":
    camera = PiCamera()
    camera.start_preview()
    sleep(10)
    camera.stop_preview()

