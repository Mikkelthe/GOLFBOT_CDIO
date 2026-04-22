import math

import cv2
import numpy as np
from pathlib import Path
from point import *

import Object_Tracking
from Object_Tracking.Course_detecter import Find_Arena
from Object_Tracking.Object_Tracking import find_objects_in_image, draw_detections_on_warp
import MoveBot.MoveBot

def find_turn(current_heading, point1, point2):
    direction_radian = np.atan2(point2.y - point1.y, point2.x - point1.x)
    target_direction = round(math.degrees(direction_radian))
    turn_flag = ""
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

#find distance between two points (fx. bot and ball)
def distance_between_points(point1: Point, point2: Point):
    return np.sqrt(np.square(point2.x - point1.x) + np.square(point2.y - point1.y))

base_path = Path(__file__).resolve().parent
images_folder = base_path.parent / "Images"
image_files = list(images_folder.glob("*.jpg"))

img = cv2.imread(image_files[0])

#find bot (start by using x=40,y=40 or something)
#bot is a circle with diameter 320mm
#find yellow ball
#determine angle from bots facing to yellow ball
#turn towards yellow ball
#Check angle is correct
#determine distance from collector to yellow ball
#drive forward to yellow ball
#Check position is correct

WARP_W, WARP_H = 800, 1200
COURT_W_CM, COURT_H_CM = 120.0, 180.0
warped = Find_Arena(img, out_w=WARP_W, out_h=WARP_H)


#temp variables to simulate bot position and heading
botX = 40
botY = 40
botCoordinates = Object_Tracking.Object_Tracking.world_cm_to_px(botX, botY, WARP_W, WARP_H)
currentHeading = 90 #measures from x-axis and goes counter-clockwise


Orangeball, whiteballs, cross = find_objects_in_image(img,WARP_W,WARP_H)

#ball coordinates
orangeX = Orangeball[3][0]
orangeY = Orangeball[3][1]
ballCoordinates = Object_Tracking.Object_Tracking.world_cm_to_px(orangeX, orangeY, WARP_W, WARP_H)
botDimensions = Object_Tracking.Object_Tracking.world_cm_to_px(32, 32, WARP_W, WARP_H)

def drawBot(x, y):
    radius = Object_Tracking.Object_Tracking.radius_cm_to_px(16)
    return cv2.circle(warped, botCoordinates, radius, (0, 0, 255), 3)


visual = warped.copy()
draw_detections_on_warp(visual,Orangeball,"position",warp_w_px=WARP_W,warp_h_px=WARP_H,court_w_cm=COURT_W_CM,court_h_cm=COURT_H_CM)
output_folder = ""
output_path = output_folder + "visTest.jpg"
cv2.imwrite(output_path, visual)





warped = drawBot(botX, botY)
warped = cv2.arrowedLine(warped, botCoordinates, ballCoordinates, (0,255,0), 3)                 # Read image
warped = cv2.resize(warped, (400, 600))
cv2.imshow("arrow", warped)
cv2.waitKey(0)
cv2.destroyAllWindows()


#For testing with actual bot
#MoveBot.MoveBot.turn(turnAngle, turnFlag)
#MoveBot.MoveBot.move_forward(dist)
