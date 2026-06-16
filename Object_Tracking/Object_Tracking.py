import cv2
import numpy as np
from pathlib import Path

from settings.courtSettings import court_settings
from .Course_detecter import CourseDetector
from settings import courtSettings

class ObjectTracker:
    def __init__(self):
        self.courseDetector = CourseDetector()
        self.accumulatedObjects = []
        self.accumulatedPriorityObjects = []
        self.validObjects = list()
        self.validPriorityObjects = list()
        self.accumulationIndex = 0


    def detect_balls_by_hsv(self, warped_bgr, lower, upper, lower2=None, upper2=None, min_area=150, max_area=600, min_circularity=0.65):
        hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
        if lower2 is None:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
        else:
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) | cv2.inRange(hsv, np.array(lower2), np.array(upper2))
        mask = cv2.erode(mask, np.ones((5,5), np.uint8), iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8), iterations=1)


        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        ballcenter = []
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
            if x < 100 or x > width-100 or y < 100 or y > height-100:
                continue
            realx, realy = self.px_to_world_cm(x, y, warp_w_px=width, warp_h_px=height)
            detections.append((float(realx), float(realy), int(x), int(y), int(r), float(area), float(circularity)))
            ballcenter.append((float(realx), float(realy)))

        # Optional: sort biggest first (often helps stability)
        detections.sort(key=lambda t: t[5], reverse=True)

        return detections, mask, ballcenter

    def draw_detections_on_warp(
        self,
        warped_bgr,
        detections,
        label_prefix,
        warp_w_px, warp_h_px,
        court_w_cm=125.0, court_h_cm=170.0,
    ):
        for i, (realx_cm, realy_cm, x_px, y_px, r_px, area, circ) in enumerate(detections):

            # Draw circle + center
            cv2.circle(warped_bgr, (x_px, y_px), r_px, (0, 255, 0), 2)   # outline
            cv2.circle(warped_bgr, (x_px, y_px), 2, (0, 255, 0), -1)     # center dot

            # Label
            text = f"{label_prefix}{i}: ({realx_cm:.1f}cm, {realy_cm:.1f}cm)"
            cv2.putText(
                warped_bgr,
                text,
                (x_px + 10, y_px - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
                cv2.LINE_AA
            )

    def draw_cross_on_warp(
        self,
        img,
        cross_data,
        warp_w_px=court_settings.image_width,
        warp_h_px=court_settings.image_height,
        court_w_cm=court_settings.court_width,
        court_h_cm=court_settings.court_height,
        border_px=100
    ):
        if cross_data is None:
            return img
        if len(cross_data) != 3:
            return img

        cv2.drawContours(img, [cross_data["vertical_box"]], 0, (0, 255, 0), 2)
        cv2.drawContours(img, [cross_data["horizontal_box"]], 0, (255, 0, 0), 2)

        cx, cy = cross_data["center"]
        cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)

        x_cm, y_cm = self.px_to_world_cm(
            cx,
            cy,
            warp_w_px=warp_w_px,
            warp_h_px=warp_h_px,
            border_px=border_px,
            court_w_cm=court_w_cm,
            court_h_cm=court_h_cm
        )

        label = f"Cross: ({x_cm:.1f}cm, {y_cm:.1f}cm)"
        cv2.putText(
            img,
            label,
            (cx + 10, cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        return img
            
    def find_objects_in_image(self, img_bgr, w, h):
        warped = self.courseDetector.find_arena(img_bgr, w, h)
        if warped is None:
            return None, None

        dilated = cv2.dilate(warped, np.ones((1,1), np.uint8), iterations=1)
        blurred = cv2.GaussianBlur(dilated, (7, 7), 0)
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        sharpened = cv2.filter2D(blurred, -1, kernel)

        orange_balls, omask, ocenter = self.detect_balls_by_hsv(blurred, lower=(0, 5, 120), upper=(40, 255, 255))
        dark_orange_balls, domask, docenter = self.detect_balls_by_hsv(warped, lower=(0, 0, 240), upper=(180, 110, 255), lower2=(0, 0, 0), upper2=(180, 100, 50))
        white_balls, wmask, wcenter = self.detect_balls_by_hsv(blurred, lower=(0, 0, 200), upper=(180, 110, 255))
        shadowywhite_balls, sw, swcenter = self.detect_balls_by_hsv(blurred, lower=(0, 0, 115), upper=(180, 100, 250))
        
        cross_position = self.courseDetector.find_red_cross_boxes(warped)

        if cross_position is None:
            print("i failed to find cross_center")
        else:
            print(len(cross_position))
            print(cross_position)

        return orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position, omask, domask, wmask, sw, wcenter, ocenter, swcenter, docenter

    def accumulate_valid_objects(self, wcenter, ocenter, swcenter, docenter):
        grouped_objects = wcenter.copy()
        grouped_objects += swcenter.copy()
        grouped_objects += docenter.copy()
        rounded_objects = list()
        for (coord_x, coord_y) in grouped_objects:
            rounded_objects.append((round(coord_x/4, 0)*4, round(coord_y/4, 0)*4))
        for (coord_x, coord_y) in rounded_objects:
            if rounded_objects.count((coord_x, coord_y)) > 1:
                # debug reporting
                # print(f"Removing {rounded_objects.count((coord_x,coord_y))-1} duplicate objects")
                rounded_objects.remove((coord_x, coord_y))

        grouped_priority_objects = ocenter.copy()
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

        # converting arrays to lists
        accumulated_objects_list = list()
        accumulated_priority_objects_list = list()
        for obj in self.accumulatedObjects:
            accumulated_objects_list += obj
        for obj in self.accumulatedPriorityObjects:
            accumulated_priority_objects_list += obj

        # filtering persistent objects
        self.validObjects = list()
        self.validPriorityObjects = list()
        for (coord_x,coord_y) in accumulated_objects_list:
            if (coord_x, coord_y) not in self.validObjects and self.accumulatedObjects.count((coord_x,coord_y)) > 2:
                self.validObjects.append((coord_x, coord_y))

        for (coord_x, coord_y) in accumulated_priority_objects_list:
            if (coord_x, coord_y) not in self.validPriorityObjects and accumulated_priority_objects_list.count((coord_x,coord_y)) > 2:
                self.accumulatedPriorityObjects.append((coord_x, coord_y))

        # print(f"{real_objects_list} FINAL LIST {len(real_objects_list)}")
        # print(f"{real_vip_objects_list} FINAL VIPS {len(real_vip_objects_list)}")

        return self.validObjects, self.validPriorityObjects