import math

import cv2
import numpy as np
from pathlib import Path
from point import *
from Object_Tracking.Object_Tracking import *
import MoveBot.MoveBot

# find the relative vector from corner to bot
# project this vector onto optimal approach vector
# calculate nearest coordinate point on optimal approach
def find_optimal_corner_approach(cornerPosition: Point, botPosition: Point):

    b = Point(0,0)

    # top left corner
    if cornerPosition.x < WARP_W/3 and cornerPosition.y < WARP_H/3:
        b = Point(1,1)
    # top right corner
    elif cornerPosition.x > WARP_W*2/3 and cornerPosition.y < WARP_H/3:
        b = Point(-1,1)
    # bottom left corner
    elif cornerPosition.x < WARP_W/3 and cornerPosition.y > WARP_H*2/3:
        b = Point(1,-1)
    # bottom right corner
    elif cornerPosition.x < WARP_W / 3 and cornerPosition.y > WARP_H * 2 / 3:
        b = Point(-1, -1)
    else:
        raise ValueError("Corner position correct")


    relative_vector = Point(botPosition.x - cornerPosition.x,
                            botPosition.y - cornerPosition.y)

    vector_factor = (relative_vector.x * b.x + relative_vector.y * b.y) / (b.x **2 + b.y **2)

    optimal_approach_vector = Point(b.x * vector_factor,
                                    b.y * vector_factor)

    optimal_position = Point(optimal_approach_vector.x - cornerPosition.x, optimal_approach_vector.y - cornerPosition.y)

    return optimal_position



#find distance between two points (for example: bot and ball)
def find_distance_between_points(point1: Point, point2: Point):
    return np.sqrt(np.square(point2.x - point1.x) + np.square(point2.y - point1.y))

#find the best turn from current heading to a point
def find_turn(current_heading, point1, point2):
    direction_radian = np.atan2(point2.y - point1.y, point2.x - point1.x)
    target_direction = round(math.degrees(direction_radian))
    delta_direction = target_direction - current_heading

    # Normalize to [-180, 180]
    delta = (delta_direction + 180) % 360 - 180
    if delta > 0:
        turn_flag = "right"
        turn_angle = delta  # Degrees to turn
    elif delta < 0:
        turn_flag = "left"
        turn_angle = -delta  # Absolute value for magnitude
    else:
        turn_flag = "none"
        turn_angle = 0

    return turn_flag, turn_angle

#find and set image folder/files
base_path = Path(__file__).resolve().parent
images_folder = base_path.parent / "Images"
image_files = list(images_folder.glob("*.jpg"))

#load image
img = cv2.imread(image_files[0])

#picture dimensions in pixel
WARP_W, WARP_H = 800, 1200

#actual dimensions in cm
COURT_W_CM, COURT_H_CM = 120.0, 180.0

#find arena in img and draw on warped
warped = find_arena(img, out_w=WARP_W, out_h=WARP_H)

#finds all objects in img
orange_ball, white_ball_list, cross = find_objects_in_image(img, WARP_W, WARP_H)

#draw objects on warped
draw_detections_on_warp(warped, orange_ball, "position", warp_w_px=WARP_W, warp_h_px=WARP_H, court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM)

#temp variables to simulate bot position and heading
bot_x = 40
bot_y = 40
botCoordinates = world_cm_to_px(bot_x, bot_y, WARP_W, WARP_H)
currentHeading = 90 #measures from x-axis and goes counter-clockwise

#ball coordinates
orange_x = orange_ball[3][0]
orange_y = orange_ball[3][1]

#convert from cm to pixel
ballCoordinates = world_cm_to_px(orange_x, orange_y, WARP_W, WARP_H)
botDimensions = world_cm_to_px(32, 32, WARP_W, WARP_H)

#draw bot on warped (current bot radius is 16)
warped = cv2.circle(warped, botCoordinates, radius_cm_to_px(16), (0, 0, 255), 3)

#draw arrow on warped
warped = cv2.arrowedLine(warped, botCoordinates, ballCoordinates, (0,255,0), 3)

#resize and show window with picture: warped
warped = cv2.resize(warped, (400, 600))
cv2.imshow("arrow", warped)
cv2.waitKey(0)
cv2.destroyAllWindows()

#save picture: warped
output_folder = ""
output_path = output_folder + "visTest.jpg"
cv2.imwrite(output_path, warped)


#For testing with actual bot
#MoveBot.MoveBot.turn(turnAngle, turnFlag)
#MoveBot.MoveBot.move_forward(dist)
