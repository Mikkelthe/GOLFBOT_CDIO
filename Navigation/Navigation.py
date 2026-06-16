import math
import cv2
import numpy as np
from pathlib import Path
from utils.point import *
from Object_Tracking.Object_Tracking import *
from settings.courtSettings import court_settings
from utils.conversion import *
class Navigation:
    def __init__(self):
        self.converter = Conversion()
        self.cd = CourseDetector()
        self.ot = ObjectTracker()
        self.warp_W = court_settings.image_width #picture width center in pixel
        self.warp_H = court_settings.image_height #picture height center in pixel
        self.buffer = 150 # distance in pixel between picture edge and the goal
        self.court_W = court_settings.court_width #arena width in cm
        self.court_H = court_settings.court_height #arena height in cm
    
    # find the relative vector from corner to bot
    # project this vector onto optimal approach vector
    # calculate nearest coordinate point on optimal approach
    def find_optimal_corner_approach(self, cornerPosition: Point, botPosition: Point):
    
        b = Point(0,0)
    
        # top left corner
        if cornerPosition.x < self.warp_W/3 and cornerPosition.y < self.warp_H/3:
            b = Point(1,1)
        # top right corner
        elif cornerPosition.x > self.warp_W*2/3 and cornerPosition.y < self.warp_H/3:
            b = Point(-1,1)
        # bottom left corner
        elif cornerPosition.x < self.warp_W/3 and cornerPosition.y > self.warp_H*2/3:
            b = Point(1,-1)
        # bottom right corner
        elif cornerPosition.x < self.warp_W / 3 and cornerPosition.y > self.warp_H * 2 / 3:
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
    def find_distance_between_points(self, point1: Point, point2: Point):
        return np.sqrt(np.square(point2.x - point1.x) + np.square(point2.y - point1.y))

    #find the best turn from current heading to a point
    def drive_to_point(self, point:Point):
        commands = []
        return commands
    
    
    #takes the bots position and heading and a destination point
    #and finds if it is best to turn left or right and by how much
    def find_turn(self, current_heading, point1, point2):
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

    #finds the optimal point to approach the goal (delivery point = 24 cm from goal)
    def find_goal_approach_point(self):
        center = Point(self.warp_W / 2, self.warp_H / 2)
        approach_point = Point(self.warp_W - self.buffer, center.y)
        return approach_point
    
    #find the robot position and heading in the picture using an ArUco-marker
    def find_bot(self, image):
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
        else:
            return None, None
        return center, angle
    
    
    
    
    
