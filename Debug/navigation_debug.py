import math
from pathlib import Path
import cv2
import numpy as np
from Navigation.Navigation import Navigation
from Object_Tracking.Course_detecter import CourseDetector
from point import Point
from Object_Tracking.Object_Tracking import ObjectTracker

nav = Navigation()
tracker = ObjectTracker()
course = CourseDetector()

# find and set image folder/files
base_path = Path(__file__).resolve().parent
images_folder = base_path.parent / "Images"
image_files = list(images_folder.glob("*.jpg"))

# load image
img = cv2.imread("../Navigation/arena.jpg")

# find arena in img and draw on warped
warped = course.find_arena(img)

# finds all objects in img
tracker.find_objects_in_image(img, nav.warp_W, nav.warp_H)
tracker.find_objects_in_image(img, nav.warp_W, nav.warp_H)
white_ball, orange_ball, cross_position = (tracker.find_objects_in_image(img, nav.warp_W, nav.warp_H))


# heading measures from x-axis and goes counter-clockwise
botCoordinates, currentHeading = tracker.find_bot(warped)

# ball coordinates
white_x = white_ball[0].x
white_y = white_ball[0].y

# convert from cm to pixel
ballCoordinates = nav.converter.world_cm_to_px(white_x, white_y, nav.warp_W, nav.warp_H)

# Calculate parallax distortion for bot
cam_height_cm = 174.0
bot_height_cm = 44.5

# scale factor to convert marker-plane radius -> ground-plane radius
scale = (cam_height_cm - bot_height_cm) / cam_height_cm  # = 1 / ratio_height

# botCoordinates is a Point returned by find_bot(warped)
# center is CENTER_POINT_WARP (Point)
CENTER_POINT_WARP = Point(nav.warp_W / 2, nav.warp_H / 2)
CENTER_POINT_CM = Point(nav.court_W / 2, nav.court_H / 2)
dx = botCoordinates.x - CENTER_POINT_WARP.x
dy = botCoordinates.y - CENTER_POINT_WARP.y

ground_dx = dx * scale
ground_dy = dy * scale

ground_x = int(round(CENTER_POINT_WARP.x + ground_dx))
ground_y = int(round(CENTER_POINT_WARP.y + ground_dy))

# Displace center to find true center from marker
displacement_in_cm = 4.5
displacement_in_px = nav.converter.cm_to_px(4.5)
angle_in_radians = math.radians(currentHeading)
ground_x = int(round(ground_x + displacement_in_px * math.cos(angle_in_radians)))
ground_y = int(round(ground_y + displacement_in_px * math.sin(angle_in_radians)))

# Update botCoordinates to the ground-projected pixel coordinates (use a Point if you prefer)
botCoordinates = Point(ground_x, ground_y)
bot_radius = nav.converter.cm_to_px(17.5, warp_w_px=nav.warp_W, warp_h_px=nav.warp_H)
bot_circle = (botCoordinates, bot_radius)
# draw bot on warped as circle(current bot radius is 16)
warped = cv2.circle(warped, botCoordinates, bot_radius, (0, 0, 255), 3)

# draw bot on warped as rectangle (width=23,5, length=34)
width_in_px = nav.converter.cm_to_px(23.5)
length_in_px = nav.converter.cm_to_px(34)
rect = botCoordinates, (length_in_px, width_in_px), currentHeading
bot_box = cv2.boxPoints(rect)
bot_box = bot_box.astype(int)
cv2.drawContours(warped, [bot_box], 0, (255, 0, 0), 3)

# draw current heading from bot on warped
arrow_length = 100  # px
end_point = Point(
    int(ground_x + arrow_length * np.cos(np.radians(currentHeading))),
    int(ground_y + arrow_length * np.sin(np.radians(currentHeading)))
)
warped = cv2.arrowedLine(warped, botCoordinates, end_point, (0, 0, 255), 3)

# draw arrow from center of bot to center of ball on warped
warped = cv2.arrowedLine(warped, botCoordinates, ballCoordinates, (0, 255, 0), 3)

# draw goal approach point
warped = cv2.circle(warped, (nav.find_goal_approach_point().x, nav.find_goal_approach_point().y), 10, (0, 0, 255), -1)

# resize and show window with picture: warped
warped = cv2.resize(warped, (600, 400))
cv2.imshow("Warped", warped)
cv2.waitKey(0)
cv2.destroyAllWindows()

# save picture: warped
output_folder = ""
output_path = output_folder + "visTest.jpg"
cv2.imwrite(output_path, warped)

# For testing with actual bot
# MoveBot.MoveBot.turn(turnAngle, turnFlag)
# MoveBot.MoveBot.move_forward(dist)
