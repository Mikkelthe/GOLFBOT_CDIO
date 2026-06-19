import math
import cv2
import numpy as np
from pathlib import Path
from utils.point import *
from utils.settings.courtSettings import court_settings
from utils.conversion import *
from utils.Linalg.vector import Vector2

class Navigation:
    def __init__(self):
        self.converter = Conversion()
        self.warp_W = court_settings.image_width #picture width center in pixel
        self.warp_H = court_settings.image_height #picture height center in pixel
        self.buffer = court_settings.padding + 50 #distance in pixel between picture edge and the goal
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
            raise ValueError("Corner position incorrect")
    
    
        relative_vector = Point(botPosition.x - cornerPosition.x,
                                botPosition.y - cornerPosition.y)
    
        vector_factor = (relative_vector.x * b.x + relative_vector.y * b.y) / (b.x **2 + b.y **2)
    
        optimal_approach_vector = Point(b.x * vector_factor,
                                        b.y * vector_factor)
    
        optimal_position = Point(optimal_approach_vector.x - cornerPosition.x, optimal_approach_vector.y - cornerPosition.y)
    
        return optimal_position
    
    #find distance between two points (for example: bot and ball)

    def find_distance_between_points(self, point1: Point, point2: Point):
        x1,y1 = self.converter.px_to_world_cm(point1.x, point1.y)
        x2,y2 = self.converter.px_to_world_cm(point2.x, point2.y)
        return np.sqrt(np.square(x2 - x1) + np.square(y2 - y1))
    
    #takes the bots position and heading and a destination point
    #and finds if it is best to turn left or right and by how much
    @staticmethod
    def find_turn(current_heading, point1, point2):
        heading_vec = Vector2(1, 0)
        heading_vec = heading_vec.rotate(current_heading)
        target_dir_vec = Vector2(point2.x - point1.x, point2.y - point1.y)
        print("\nrobot position: " + str(point1.x) + ", " + str(point1.y) + "\n")
        print("\npunkt vi kører efter: " + str(point2.x) + ", " + str(point2.y) + "\n")
        direction_radian = Vector2.signedAngle(heading_vec, target_dir_vec)
        print("Direction radian: " + str(direction_radian) + "\n")
        if direction_radian < 0:
            turn_flag = "right"
            turn_angle = abs(direction_radian)  # Degrees to turn
        elif direction_radian > 0:
            turn_flag = "left"
            turn_angle = abs(direction_radian)  # Absolute value for magnitude
        else:
            turn_flag = "none"
            turn_angle = 0


        return turn_flag, turn_angle

    #finds the optimal point to approach the goal (delivery point = 24 cm from goal)
    def find_goal_approach_point(self):
        center = Point(self.warp_W / 2, self.warp_H / 2)
        goal_point = Point(self.warp_W - self.buffer, center.y)
        #TODO adjust values to match actual points for delivery (needs testing)
        approach_point = Point(goal_point.x - self.converter.cm_to_px(30), center.y)
        delivery_point = Point(goal_point.x - self.converter.cm_to_px(20), center.y)
        return approach_point, delivery_point
    
    
    
    
    
