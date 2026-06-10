import math
import cv2
import numpy as np
from pathlib import Path
from point import *
from Object_Tracking.Object_Tracking import *

#kunne vi holde os til px så vidt som muligt og så kun konvertere til cm ved behov?
#kan ikke se AruCo ved kanten
#bane/pixel problem?

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

#find the robot position and heading in the picture using an ArUco-marker
def find_bot(image):
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    detector = cv2.aruco.ArucoDetector(aruco_dict)

    corners, ids, rejected = detector.detectMarkers(image)

    if ids is not None:
        pts = corners[0][0]

        center = Point(*np.mean(pts, axis=0))

        # Marker top edge
        top_left = pts[0]
        top_right = pts[1]

        heading = top_right - top_left

        angle = np.degrees(
            np.arctan2(heading[1], heading[0])
        )
    return center, angle


#find and set image folder/files
base_path = Path(__file__).resolve().parent
images_folder = base_path.parent / "Images"
image_files = list(images_folder.glob("*.jpg"))

#load image
img = cv2.imread("arena3.jpg")

#picture dimensions center in pixel
WARP_W, WARP_H = 1100, 700
CENTER_POINT_WARP = Point(WARP_W / 2, WARP_H / 2)

#actual dimensions and center in cm
COURT_W_CM, COURT_H_CM = 170.0, 125.0
CENTER_POINT_CM = Point(COURT_W_CM / 2, COURT_H_CM / 2)

#find arena in img and draw on warped
warped = find_arena(img, out_w=WARP_W, out_h=WARP_H)

#finds all objects in img
orange_ball, white_ball, cross = find_objects_in_image(img, WARP_W, WARP_H)

#draw objects on warped
draw_detections_on_warp(warped, orange_ball, "position", warp_w_px=WARP_W, warp_h_px=WARP_H, court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM)
draw_detections_on_warp(warped, white_ball, "position", warp_w_px=WARP_W, warp_h_px=WARP_H, court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM)

#heading measures from x-axis and goes counter-clockwise
botCoordinates, currentHeading = find_bot(warped)
#botCoordinates = botCoordinates.x, botCoordinates.y

#ball coordinates
white_x = white_ball[0][0]
white_y = white_ball[0][1]

#convert from cm to pixel
ballCoordinates = world_cm_to_px(white_x, white_y, WARP_W, WARP_H)
botDimensions = world_cm_to_px(32, 32, WARP_W, WARP_H)

#Calculate parallax distortion for bot
cam_height_cm = 195
cam_height_px = cm_to_px(cam_height_cm)
bot_height_cm = 46
bot_height_px = cm_to_px(bot_height_cm)
ratio_height = cam_height_px / (cam_height_px - bot_height_px)

center_to_bot_dist = find_distance_between_points(CENTER_POINT_WARP, botCoordinates)
x_ratio = int(botCoordinates.x / ratio_height)
y_ratio = int(botCoordinates.y / ratio_height)
botCoordinates = x_ratio, y_ratio


#draw bot on warped (current bot radius is 16)
warped = cv2.circle(warped, botCoordinates, cm_to_px(16), (0, 0, 255), 3)

#draw current heading from bot on warped
arrow_length = 100 #px
end_point = (
    int(x_ratio + arrow_length * np.cos(np.radians(currentHeading))),
    int(y_ratio + arrow_length * np.sin(np.radians(currentHeading)))
)
warped = cv2.arrowedLine(warped, botCoordinates, end_point, (0,0,255), 3)

#draw arrow from center of bot to center of ball on warped
warped = cv2.arrowedLine(warped, botCoordinates, ballCoordinates, (0,255,0), 3)

#resize and show window with picture: warped
warped = cv2.resize(warped, (600, 400))
cv2.imshow("Warped", warped)
cv2.waitKey(0)
cv2.destroyAllWindows()

#save picture: warped
output_folder = ""
output_path = output_folder + "visTest.jpg"
cv2.imwrite(output_path, warped)


#For testing with actual bot
#MoveBot.MoveBot.turn(turnAngle, turnFlag)
#MoveBot.MoveBot.move_forward(dist)
