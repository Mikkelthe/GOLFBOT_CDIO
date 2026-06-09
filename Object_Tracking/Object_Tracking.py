import cv2
import numpy as np
from pathlib import Path
from .Course_detecter import find_arena
from .Course_detecter import find_red_cross_center
from .Course_detecter import find_red_cross_boxes


def detect_balls_by_hsv(warped_bgr, lower, upper, min_area=150, max_area=400, min_circularity=0.70):
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    mask = cv2.erode(mask, np.ones((5,5), np.uint8), iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8), iterations=1)


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
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
        realx, realy = px_to_world_cm(x, y, warp_w_px=warped_bgr.shape[1], warp_h_px=warped_bgr.shape[0])
        detections.append((float(realx), float(realy), int(x), int(y), int(r), float(area), float(circularity)))

    # Optional: sort biggest first (often helps stability)
    detections.sort(key=lambda t: t[5], reverse=True)

    return detections, mask

#Coordinates
def px_to_world_cm(x_px, y_px, warp_w_px, warp_h_px, court_w_cm=120.0, court_h_cm=180.0):
    cm_per_px_x = court_w_cm / warp_w_px
    cm_per_px_y = court_h_cm / warp_h_px

    x_cm = x_px * cm_per_px_x
    y_cm_from_top = y_px * cm_per_px_y
    y_cm_from_bottom = court_h_cm - y_cm_from_top
    return x_cm, y_cm_from_bottom

#Coordinates
def world_cm_to_px(x_cm, y_cm, warp_w_px=800, warp_h_px=1200, court_w_cm=120.0, court_h_cm=180.0):
    cm_per_px_x = court_w_cm / warp_w_px
    cm_per_px_y = court_h_cm / warp_h_px

    x_px = x_cm / cm_per_px_x
    y_px_from_top = y_cm / cm_per_px_y
    y_px_from_bottom = warp_h_px - y_px_from_top
    return int(x_px), int(y_px_from_bottom)

#Requires court to be uniform to work correctly
def radius_cm_to_px(radius_cm, warp_w_px=800, warp_h_px=1200, court_w_cm=120.0, court_h_cm=180.0):
    cm_per_px_x = court_w_cm / warp_w_px
    return int(radius_cm / cm_per_px_x)

def draw_detections_on_warp(
    warped_bgr,
    detections,
    label_prefix,
    warp_w_px, warp_h_px,
    court_w_cm=120.0, court_h_cm=180.0,
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
        
def draw_cross_on_warp(img, cross_data,
                       warp_w_px, warp_h_px,
                       court_w_cm, court_h_cm):
    if cross_data is None:
        return img
    if len(cross_data) != 3:
        return img

    cv2.drawContours(img, [cross_data["vertical_box"]], 0, (0, 255, 0), 2)
    cv2.drawContours(img, [cross_data["horizontal_box"]], 0, (255, 0, 0), 2)

    cx, cy = cross_data["center"]
    cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)

    x_cm = cx * court_w_cm / warp_w_px
    y_cm = cy * court_h_cm / warp_h_px

    label = f"Cross: ({x_cm:.1f}cm, {y_cm:.1f}cm)"
    cv2.putText(img, label, (cx + 10, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    return img
        
def find_objects_in_image(img_bgr,w,h):
    warped = find_arena(img_bgr, w, h)
    if warped is None:
        return None, None

    orange_balls, omask = detect_balls_by_hsv(warped, lower=(5,120,120), upper=(25,255,255))
    white_balls, wmask   = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))
    
    cross_position = find_red_cross_boxes(warped)

    if cross_position is None:
        print("i failed to find cross_center")
    else:
        print(len(cross_position))
        print(cross_position)

    return orange_balls, white_balls, cross_position
