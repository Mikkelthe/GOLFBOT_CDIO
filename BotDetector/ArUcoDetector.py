import cv2
import numpy as np
from Navigation.point import Point


def find_bot(image):
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_4X4_50
    )

    detector = cv2.aruco.ArucoDetector(aruco_dict)

    corners, ids, rejected = detector.detectMarkers(image)

    if ids is not None:
        pts = corners[0][0]

        center = Point(np.mean(pts, axis=0))

        # Marker top edge
        top_left = pts[0]
        top_right = pts[1]

        heading = top_right - top_left

        angle = np.degrees(
            np.arctan2(heading[1], heading[0])
        )
        #for debugging
        #print("Center:", center)
        #print("Orientation:", angle)
    return center, angle

#image = cv2.imread("arena3.jpg")
#find_bot(image)
