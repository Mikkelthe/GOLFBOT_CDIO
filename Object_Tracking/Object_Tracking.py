import cv2
import numpy as np
from pathlib import Path
from .Course_detecter import Find_Arena
from .Course_detecter import find_red_cross_center


def detect_balls_by_hsv(warped_bgr, lower, upper, min_area=80, max_area=2000, min_circularity=0.75):
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
    mask = cv2.erode(mask, np.ones((5, 5), np.uint8), iterations=1)
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

def px_to_world_cm(x_px, y_px, warp_w_px, warp_h_px, court_w_cm=120.0, court_h_cm=180.0):
    cm_per_px_x = court_w_cm / warp_w_px
    cm_per_px_y = court_h_cm / warp_h_px

    x_cm = x_px * cm_per_px_x
    y_cm_from_top = y_px * cm_per_px_y
    y_cm_from_bottom = court_h_cm - y_cm_from_top
    return x_cm, y_cm_from_bottom

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
        
def draw_cross_on_warp(
    warped_bgr,
    cross_px,
    warp_w_px,
    warp_h_px,
    court_w_cm=120.0,
    court_h_cm=180.0,
):
    if cross_px is None:
        return

    x_px, y_px = cross_px
    x_cm, y_cm = px_to_world_cm(
        x_px, y_px,
        warp_w_px=warp_w_px,
        warp_h_px=warp_h_px,
        court_w_cm=court_w_cm,
        court_h_cm=court_h_cm
    )

    cv2.circle(warped_bgr, (x_px, y_px), 8, (0, 0, 255), -1)
    cv2.circle(warped_bgr, (x_px, y_px), 16, (0, 0, 255), 2)

    text = f"C: ({x_cm:.1f}cm, {y_cm:.1f}cm)"
    cv2.putText(
        warped_bgr,
        text,
        (x_px + 10, y_px - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA
    )
        
def find_objects_in_image(img_bgr,w,h):
    warped = Find_Arena(img_bgr,w,h)
    if warped is None:
        return None, None

    orange_balls, omask = detect_balls_by_hsv(warped, lower=(5,120,120), upper=(25,255,255))
    white_balls, wmask   = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))
    
    cross_px = find_red_cross_center(warped)
    
    if cross_px is not None:
        cross_x_cm, cross_y_cm = px_to_world_cm(
            cross_px[0],
            cross_px[1],
            warp_w_px=warped.shape[1],
            warp_h_px=warped.shape[0]
        )
        cross_position = (cross_x_cm, cross_y_cm, cross_px[0], cross_px[1])
    else:
        cross_position = None

    return orange_balls, white_balls, cross_position

# Used to generate pictures to see the results of the detection
# Not used in the actual bot, but can be useful for debugging and tuning parameters
"""if __name__ == "__main__":
    base_path = Path(__file__).resolve().parent
    images_folder = base_path.parent / "Images"
    output_folder = base_path.parent / "Warped_Images"

    output_folder.mkdir(exist_ok=True)

    # ---- Court settings ----
    WARP_W, WARP_H = 800, 1200
    COURT_W_CM, COURT_H_CM = 120.0, 180.0

    image_files = list(images_folder.glob("*.jpg"))

    print(f"Found {len(image_files)} images")

    for img_path in image_files:
        print(f"Processing {img_path.name}")

        img = cv2.imread(str(img_path))
        if img is None:
            print("Could not load image")
            continue

        warped = Find_Arena(img, out_w=WARP_W, out_h=WARP_H)
        if warped is None:
            raise RuntimeError("Could not find arena")

        orange_balls, omask = detect_balls_by_hsv(warped, lower=(5,120,120), upper=(25,255,255))
        white_balls, wmask   = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))
        cross_position = find_red_cross_center(warped)
        print(len(orange_balls),len(white_balls))
        vis = warped.copy()
        draw_detections_on_warp(
            vis, orange_balls, "O",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_detections_on_warp(
            vis, white_balls, "W",
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )
        draw_cross_on_warp(
            vis, cross_position,
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
        )

        # Show results
        output_path = output_folder / img_path.name
        cv2.imwrite(str(output_path), vis)"""
