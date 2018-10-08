from geometry_msgs.msg import Point

# Game runs for 5 minutes
TOTAL_ALLOWED_TIME = 300 # Seconds

# Attribute of camera image
PICTURE_SIZE = (1280, 960)

# Sites may need to adjust the following 4 settings to match their
# arena and camera positions
ORIGIN_PIXELS = Point(619, 501, 0)
RED_BASE = Point(390, 749, 0)
BLUE_BASE = Point(862, 249, 0)
ARENA_WIDTH_PIXELS = 828        # assumed square

# These should be good enough if arena built-to-spec
ARENA_WIDTH_MM = 1145           # assumed square
COVERT_PIXEL2MM = ARENA_WIDTH_MM / ARENA_WIDTH_PIXELS
