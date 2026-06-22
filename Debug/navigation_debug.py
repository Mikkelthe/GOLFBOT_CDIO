import math
from pathlib import Path
import cv2
import numpy as np
from Navigation.Navigation import Navigation
from Object_Tracking.Course_detecter import CourseDetector
from utils.point import Point
from Object_Tracking.Object_Tracking import ObjectTracker

if __name__ == '__main__':
    nav = Navigation()
    tracker = ObjectTracker()
    course = CourseDetector()

    # find and set image folder/files
    base_path = Path(__file__).resolve().parent
    images_folder = base_path.parent / "Images"
    image_files = list(images_folder.glob("*.jpg"))

    # load image
    #img = cv2.imread("../Navigation/arena.jpg")

    #load video device
    videodevice = cv2.VideoCapture(0,cv2.CAP_DSHOW)
    videodevice.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    videodevice.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    _,img = videodevice.read()

    # find arena in img and draw on warped
    warped = course.find_arena(img)

    # finds all objects in img
    white_ball, orange_ball, cross_position = (tracker.find_objects_in_image(videodevice))


    # heading measures from x-axis and goes counter-clockwise
    botCoordinates, currentHeading = tracker.find_bot(warped)

    # ball coordinates
    white_x = white_ball[0].x
    white_y = white_ball[0].y
    ballCoordinates = (white_x, white_y)

    bot_radius = nav.converter.cm_to_px(17.5, warp_w_px=nav.warp_W, warp_h_px=nav.warp_H)
    bot_circle = (botCoordinates, bot_radius)
    # draw bot on warped as circle(current bot radius is 16)
    warped = cv2.circle(warped, botCoordinates, bot_radius, (0, 0, 255), 3)

    # draw bot on warped as rectangle (width=23,5, length=34)
    width_in_px = nav.converter.cm_to_px(23.5)
    length_in_px = nav.converter.cm_to_px(34)
    rect = botCoordinates, (length_in_px, width_in_px), np.rad2deg(currentHeading)
    bot_box = cv2.boxPoints(rect)
    bot_box = bot_box.astype(int)
    cv2.drawContours(warped, [bot_box], 0, (255, 0, 0), 3)

    # draw current heading from bot on warped
    arrow_length = 100  # px
    end_point = Point(
        int(botCoordinates.x + arrow_length * np.cos(currentHeading)),
        int(botCoordinates.y + arrow_length * np.sin(currentHeading))
    )
    warped = cv2.arrowedLine(warped, botCoordinates, end_point, (0, 0, 255), 3)

    # draw arrow from center of bot to center of ball on warped
    warped = cv2.arrowedLine(warped, botCoordinates, ballCoordinates, (0, 255, 0), 3)

    # draw goal approach point
    approach, deliver = nav.find_goal_approach_point()
    warped = cv2.circle(warped, approach, 10, (0, 0, 255), -1)

    # draw goal delivery point
    warped = cv2.circle(warped, deliver, 10, (0, 0, 255), -1)

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
