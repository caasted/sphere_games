import cv2
import host.constants as constants
import numpy as np
from geometry_msgs.msg import Point

def pixels_2_mm(pt_px):
    x = (pt_px.x - constants.ORIGIN_PIXELS.x) * constants.COVERT_PIXEL2MM
    y = -1 * ((pt_px.y - constants.ORIGIN_PIXELS.y) * constants.COVERT_PIXEL2MM)
    z = (constants.ORIGIN_PIXELS.z - pt_px.z) * constants.COVERT_PIXEL2MM

    return Point(int(x),int(y),int(z))

def mm_2_pixel(pt_mm):
    x = constants.ORIGIN_PIXELS.x + (pt_mm.x * constants.COVERT_MM2PIXEL)
    y = constants.ORIGIN_PIXELS.y - (pt_mm.y * constants.COVERT_MM2PIXEL)
    z = constants.ORIGIN_PIXELS.z + (pt_mm.z * constants.COVERT_MM2PIXEL)

    return Point(int(x),int(y),int(z))

def calculate_distance(start, end):
    if(start is None):
        return

    if(end is None):
        return

    distance = np.sqrt((start.x - end.x) ** 2 +
                       (start.y - end.y) ** 2)

    return distance

def calculate_error_heading(start, end, positive_only=False):

    if(start is None):
        return

    if(end is None):
        return

    x = end.x - start.x
    y = end.y - start.y

    theta = np.arctan2(y,x)

    angle = 90 - np.rad2deg(theta) # Rotate so heading is from true north

    if(angle > 180):
        angle = angle - 360
    elif(angle < -180):
        angle = angle + 360

    if(positive_only):
        angle = (720 + angle)%360

    return angle

def update_arena(game_state, time_elapsed, score, center, base, flag, img):
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
    if (game_state == 0):
        game_text = "Waiting"
    elif (game_state == 1):
        game_text = "Running"
    elif (game_state == 2):
        game_text = "Game Over"
    else:
        game_text = "ERROR"

    cv2.putText(arena_img, 'Status: ' + game_text,
                (10, 870),
                font,
                fontScale,
                fontColor,
                lineType)

    # Time Remaining
    cv2.putText(arena_img, 'Time Remaining: ' + str(constants.TOTAL_ALLOWED_TIME - time_elapsed) + ' s',
                (10, 910),
                font,
                fontScale,
                fontColor,
                lineType)

    # Position Information
    cv2.putText(arena_img, 'Red (' + str(center['red'].x) + ', ' + str(center['red'].y) + ')',
                (1000, 40),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    cv2.putText(arena_img, 'Blue (' + str(center['blue'].x) + ', ' + str(center['blue'].y) + ')',
                (600, 40),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    # Score Information
    cv2.putText(arena_img, 'Red Team: ' + str(score['red']),
                (1050, 910),
                font,
                fontScale,
                (0, 0, 255),
                lineType)

    cv2.putText(arena_img, 'Blue Team: ' + str(score['blue']),
                (780, 910),
                font,
                fontScale,
                (255, 0, 0),
                lineType)

    # Center
    cv2.circle(arena_img, (constants.ORIGIN_PIXELS.x, constants.ORIGIN_PIXELS.y), 3, (255, 255, 255), -1)

    # Sphero Locations
    if (flag['red']):
        thickness = -1
    else:
        thickness = 2

    cv2.circle(arena_img, (center['red'].x, center['red'].y), 10, (0, 0, 255), thickness=thickness)

    if (flag['blue']):
        thickness = -1
    else:
        thickness = 2

    cv2.circle(arena_img, (center['blue'].x, center['blue'].y), 10, (255, 0, 0), thickness=thickness)

    # Base Locations
    if (not flag['red']):
        thickness = -1
    else:
        thickness = 2

    cv2.circle(arena_img, (base['red'].x, base['red'].y), 10, (0, 0, 255), thickness=thickness)

    if (not flag['blue']):
        thickness = -1
    else:
        thickness = 2

    cv2.circle(arena_img, (base['blue'].x, base['blue'].y), 10, (255, 0, 0), thickness=thickness)

    return arena_img
