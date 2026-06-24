import cv2
import numpy as np
import math

from golfbot.utils import Conversion, Point, Angle
from golfbot.utils.settings import court_settings
from ._courseDetector import CourseDetector

class ObjectTracker:
    def __init__(self):
        self.courseDetector = CourseDetector()
        self.accumulatedObjects = []
        self.accumulatedPriorityObjects = []
        self.validObjects = list()
        self.validPriorityObjects = list()
        self.crossPosition = tuple()
        self.accumulationIndex = 0
        self.conversion = Conversion()


    def __detect_balls_by_hsv(self, warped_bgr, lower, upper, lower2=None, upper2=None, min_area=150, max_area=600, min_circularity=0.65):
        hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
        if lower2 is None:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        else:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) | cv2.inRange(hsv, np.array(lower2), np.array(upper2))
        mask = cv2.erode(mask, np.ones((5,5), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8), iterations=1)


        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        ball_center = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area or area > max_area:
                continue

            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)
            if circularity < min_circularity:
                continue

            (x, y), r = cv2.minEnclosingCircle(c)

            width = warped_bgr.shape[1]
            height = warped_bgr.shape[0]
            if x < 90 or x > width-90 or y < 90 or y > height-90:
                continue
            real_x, real_y = self.conversion.px_to_world_cm(x, y, warp_w_px=width, warp_h_px=height)
            detections.append((float(real_x), float(real_y), int(x), int(y), int(r), float(area), float(circularity)))
            ball_center.append((float(real_x), float(real_y)))

        # Optional: sort biggest first (often helps stability)
        detections.sort(key=lambda t: t[5], reverse=True)

        return detections, mask, ball_center

    def find_bot(self, image):
        image = self.courseDetector.find_arena(image)
        cam_height_cm = 190
        bot_height_cm = 45

        scale = (cam_height_cm - bot_height_cm) / cam_height_cm

        aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        detector = cv2.aruco.ArucoDetector(aruco_dict)

        corners, ids, rejected = detector.detectMarkers(image)

        if ids is not None:
            pts = corners[0][0]

            center = Point(*np.mean(pts, axis=0))

            # Marker top edge
            top_left = pts[3]
            top_right = pts[2]

            heading = [top_right[0] - top_left[0],top_left[1] - top_right[1]]

            angle = (
                np.arctan2(heading[1], heading[0])
            )
            angle_in_radians = angle

            CENTER_POINT_WARP = Point(court_settings.image_width / 2, court_settings.image_height / 2)
            CENTER_POINT_CM = Point(court_settings.court_width / 2, court_settings.court_height / 2)
            #Mikkels quick fix
            dx = center.x - CENTER_POINT_WARP.x
            dy = center.y - CENTER_POINT_WARP.y

            ground_dx = dx*scale
            ground_dy = dy*scale

            ground_x = int(round(CENTER_POINT_WARP.x + ground_dx))
            ground_y = int(round(CENTER_POINT_WARP.y + ground_dy))

            # Displace center to find true center from marker
            displacement_x_px= int(round(34.4 * math.cos(angle_in_radians)))
            displacement_y_px = int(round(28.8 * math.sin(angle_in_radians)))
            #displacement_x_px, displacement_y_px = self.conversion.world_cm_to_px(displacement_x, displacement_y)
            newcoursettings = court_settings
            # Update botCoordinates to the ground-projected pixel coordinates (use a Point if you prefer)
            bot_coordinates = Point(ground_x + displacement_x_px, ground_y + displacement_y_px)

            center = bot_coordinates
        else:
            return None, None
        return center, angle_in_radians

    def find_objects_in_image(self, video_device: cv2.VideoCapture):
        i = 0
        while i < 5:
            _,img = video_device.read()
            cv2.imwrite("Frame.jpg", img)
            warped = self.courseDetector.find_arena(img)
            if warped is None:
                return None, None

            dilated = cv2.dilate(warped, np.ones((1,1), np.uint8), iterations=1)
            blurred = cv2.GaussianBlur(dilated, (7, 7), 0)
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
            sharpened = cv2.filter2D(blurred, -1, kernel)

            orange_balls, o_mask, o_center = self.__detect_balls_by_hsv(blurred, lower=(0, 5, 120), upper=(40, 255, 255))
            unblurred_white_balls, ubw_mask, ubw_center = self.__detect_balls_by_hsv(warped, lower=(0, 0, 240), upper=(180, 110, 255), lower2=(0, 0, 0), upper2=(180, 100, 50))
            white_balls, w_mask, w_center = self.__detect_balls_by_hsv(blurred, lower=(0, 0, 200), upper=(180, 110, 255))
            shadowy_white_balls, sw, sw_center = self.__detect_balls_by_hsv(blurred, lower=(0, 0, 115), upper=(180, 100, 250))

            self.crossPosition = self.courseDetector.find_red_cross_boxes(warped)

            if self.crossPosition is None:
                print("i failed to find cross_center")
            else:
                print(len(self.crossPosition))
                print(self.crossPosition)

            # Two methods collapsed into one
            # return orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, self.crossPosition, omask, domask, wmask, sw, wcenter, ocenter, swcenter, docenter
            # def accumulate_valid_objects(self, wcenter, ocenter, swcenter, docenter):

            grouped_objects = w_center.copy()
            grouped_objects += sw_center.copy()
            grouped_objects += ubw_center.copy()
            rounded_objects = list()
            for (coord_x, coord_y) in grouped_objects:
                rounded_objects.append((round(coord_x/4, 0)*4, round(coord_y/4, 0)*4))
            for (coord_x, coord_y) in rounded_objects:
                if rounded_objects.count((coord_x, coord_y)) > 1:
                    # debug reporting
                    # print(f"Removing {rounded_objects.count((coord_x,coord_y))-1} duplicate objects")
                    rounded_objects.remove((coord_x, coord_y))

            grouped_priority_objects = o_center.copy()
            rounded_priority_objects = list()
            for (coord_x, coord_y) in grouped_priority_objects:
                rounded_priority_objects.append((round(coord_x/4, 0)*4, round(coord_y/4, 0)*4))
            for (coord_x, coord_y) in rounded_priority_objects:
                if rounded_priority_objects.count((coord_x, coord_y)) > 1:
                    # debug reporting
                    # print(f"Removing {rounded_priority_objects.count((coord_x,coord_y))-1} duplicate vip objects")
                    rounded_priority_objects.remove((coord_x, coord_y))

            # debug reporting
            # print(f"i found {rounded_objects}. That's {len(rounded_objects)} balls")
            # print(f"i found {rounded_priority_objects}. That's {len(rounded_vip_objects)} super balls")
            if len(self.accumulatedObjects) < 5:
                self.accumulatedObjects.append(rounded_objects)
            else:
                self.accumulatedObjects[self.accumulationIndex % 5] = rounded_objects

            if len(self.accumulatedPriorityObjects) < 5:
                self.accumulatedPriorityObjects.append(rounded_priority_objects)
            else:
                self.accumulatedPriorityObjects[self.accumulationIndex % 5] = rounded_priority_objects

            self.accumulationIndex += 1
            self.accumulationIndex = self.accumulationIndex % 5

            i += 1

        # converting arrays to lists
        accumulated_objects_list = list()
        accumulated_priority_objects_list = list()
        for obj in self.accumulatedObjects:
            accumulated_objects_list += obj
        for obj in self.accumulatedPriorityObjects:
            accumulated_priority_objects_list += obj

        # debug reporting
        # print(f"accumulated_objects_list: {accumulated_objects_list}")
        # print(f"accumulated_objects_list: {accumulated_priority_objects_list}")

        # filtering persistent objects
        valid_objects = list()
        valid_priority_objects = list()
        self.validObjects = list()
        self.validPriorityObjects = list()
        for (coord_x, coord_y) in accumulated_objects_list:
            if (coord_x, coord_y) not in valid_objects and accumulated_objects_list.count((coord_x,coord_y)) > 2:
                valid_objects.append((coord_x,coord_y))
                coord_x_px, coord_y_px = self.conversion.world_cm_to_px(coord_x, coord_y)
                coord_point = Point(coord_x_px, coord_y_px)
                self.validObjects.append(coord_point)

        for (coord_x, coord_y) in accumulated_priority_objects_list:
            if (coord_x, coord_y) not in valid_priority_objects and accumulated_priority_objects_list.count((coord_x,coord_y)) > 2:
                valid_priority_objects.append((coord_x, coord_y))
                print(f"accumulated_objects_list: {valid_priority_objects}")
                coord_x_px, coord_y_px = self.conversion.world_cm_to_px(coord_x, coord_y)
                coord_point = Point(coord_x_px, coord_y_px)
                self.validPriorityObjects.append(coord_point)

        # print(f"{real_objects_list} FINAL LIST {len(real_objects_list)}")
        # print(f"{real_vip_objects_list} FINAL VIPS {len(real_vip_objects_list)}")

        return self.validObjects, self.validPriorityObjects, self.crossPosition