import cv2
import numpy as np
from pathlib import Path
from .Course_detecter import find_arena
from .Course_detecter import find_red_cross_center
from .Course_detecter import find_red_cross_boxes


def detect_balls_by_hsv(warped_bgr, lower, upper, lower2=None, upper2=None, min_area=125, max_area=800, min_circularity=0.75):
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    if lower2 is None:
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    else:
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper)) | cv2.inRange(hsv, np.array(lower2), np.array(upper2))
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

        width = warped_bgr.shape[1]
        height = warped_bgr.shape[0]
        if x < 100 or x > width-100 or y < 100 or y > height-100:
            continue
        realx, realy = px_to_world_cm(x, y, warp_w_px=width, warp_h_px=height)
        detections.append((float(realx), float(realy), int(x), int(y), int(r), float(area), float(circularity)))

    # Optional: sort biggest first (often helps stability)
    detections.sort(key=lambda t: t[5], reverse=True)

    return detections, mask

#Coordinates
def px_to_world_cm(
    x_px,
    y_px,
    warp_w_px,
    warp_h_px,
    border_px=100,
    court_w_cm=170.0,
    court_h_cm=125.0
):
    court_w_px = warp_w_px - 2 * border_px
    court_h_px = warp_h_px - 2 * border_px

    # Convert image pixel coordinate to court-local pixel coordinate
    x_local_px = x_px - border_px
    y_local_px = y_px - border_px

    cm_per_px_x = court_w_cm / court_w_px
    cm_per_px_y = court_h_cm / court_h_px

    x_cm = x_local_px * cm_per_px_x

    # y = 0 at bottom of court
    y_cm_from_top = y_local_px * cm_per_px_y
    y_cm = court_h_cm - y_cm_from_top

    return x_cm, y_cm

#Coordinates
def world_cm_to_px(
    x_cm,
    y_cm,
    img_w_px,
    img_h_px,
    border_px=100,
    court_w_cm=170.0,
    court_h_cm=125.0
):
    court_w_px = img_w_px - 2 * border_px
    court_h_px = img_h_px - 2 * border_px

    cm_per_px_x = court_w_cm / court_w_px
    cm_per_px_y = court_h_cm / court_h_px

    x_local_px = x_cm / cm_per_px_x

    # y_cm is measured from bottom, but image y is measured from top
    y_px_from_top_inside_court = (court_h_cm - y_cm) / cm_per_px_y

    # Add the 50 px border back
    x_px = x_local_px + border_px
    y_px = y_px_from_top_inside_court + border_px

    return int(round(x_px)), int(round(y_px))

#Requires court to be uniform to work correctly
def cm_to_px(
    radius_cm,
    warp_w_px=1500,
    warp_h_px=1000,
    border_px=100,
    court_w_cm=170.0,
    court_h_cm=125.0
):
    court_w_px = warp_w_px - 2 * border_px
    cm_per_px_x = court_w_cm / court_w_px
    return int(round(radius_cm / cm_per_px_x))

def draw_detections_on_warp(
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
    img,
    cross_data,
    warp_w_px,
    warp_h_px,
    court_w_cm=125.0,
    court_h_cm=170.0,
    border_px=50
):
    if cross_data is None:
        return img
    if len(cross_data) != 3:
        return img

    cv2.drawContours(img, [cross_data["vertical_box"]], 0, (0, 255, 0), 2)
    cv2.drawContours(img, [cross_data["horizontal_box"]], 0, (255, 0, 0), 2)

    cx, cy = cross_data["center"]
    cv2.circle(img, (cx, cy), 5, (0, 255, 255), -1)

    x_cm, y_cm = px_to_world_cm(
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
        
def find_objects_in_image(img_bgr,w,h):
    warped = find_arena(img_bgr, w, h)
    if warped is None:
        return None, None

    dilated = cv2.dilate(warped, np.ones((3,3), np.uint8), iterations=1)
    blurred = cv2.medianBlur(dilated, 5)
    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    sharpened = cv2.filter2D(blurred, -1, kernel)

    orange_balls, omask = detect_balls_by_hsv(sharpened, lower=(0, 5, 120), upper=(40, 255, 255), lower2=(0, 0, 0), upper2=(180, 30, 50))
    dark_orange_balls, domask = detect_balls_by_hsv(sharpened, lower=(5, 120, 120), upper=(30, 255, 255), lower2=(0, 0, 0), upper2=(180, 30, 50))
    white_balls, wmask = detect_balls_by_hsv(sharpened, lower=(0, 0, 180), upper=(180, 90, 255), lower2=(0, 0, 0), upper2=(180, 30, 50))
    shadowywhite_balls, sw = detect_balls_by_hsv(sharpened, lower=(0, 0, 140), upper=(180, 90, 250), lower2=(0, 0, 0), upper2=(180, 30, 75))
    
    cross_position = find_red_cross_boxes(warped)

    if cross_position is None:
        print("i failed to find cross_center")
    else:
        print(len(cross_position))
        print(cross_position)

    return orange_balls, white_balls, dark_orange_balls, shadowywhite_balls, cross_position, omask, domask, wmask, sw
