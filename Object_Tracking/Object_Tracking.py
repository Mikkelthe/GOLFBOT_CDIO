import cv2
import numpy as np
from pathlib import Path
from Course_detecter import Find_Arena

def hsv_mask_red(hsv):
    # red wraps hue -> two ranges
    lower1 = np.array([0, 80, 60])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 80, 60])
    upper2 = np.array([180, 255, 255])
    return cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

def order_corners(pts):
    # pts: (4,2)
    pts = np.array(pts, dtype=np.float32)
    s = pts.sum(axis=1)
    diff = np.diff(pts, axis=1).reshape(-1)
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(diff)]
    bl = pts[np.argmax(diff)]
    return np.array([tl, tr, br, bl], dtype=np.float32)

def find_arena_and_warp(img, out_w=800, out_h=1200):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = hsv_mask_red(hsv)

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, None, mask

    c = max(contours, key=cv2.contourArea)
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.02 * peri, True)

    # If approx isn't 4 points, you can fall back to minAreaRect
    if len(approx) != 4:
        rect = cv2.minAreaRect(c)
        approx = cv2.boxPoints(rect).astype(np.int32).reshape(-1, 1, 2)

    corners = order_corners(approx.reshape(-1, 2))
    dst = np.array([[0,0],[out_w,0],[out_w,out_h],[0,out_h]], dtype=np.float32)

    M = cv2.getPerspectiveTransform(corners, dst)
    warped = cv2.warpPerspective(img, M, (out_w, out_h))
    return warped, M, mask

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

def find_red_cross_center(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    red_mask = hsv_mask_red(hsv)

    moments = cv2.moments(red_mask)
    if moments["m00"] == 0:
        return None

    x = int(moments["m10"] / moments["m00"])
    y = int(moments["m01"] / moments["m00"])

#Code that detects Orange balls close to each other for later use if white balls
#start becoming blobs aswell
"""def detect_orange_balls_hough(
    warped_bgr,
    lower=(5, 120, 120),
    upper=(25, 255, 255),
    dp=1.2,
    minDist=25,
    param1=120,
    param2=14,
    minRadius=8,
    maxRadius=20
):
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

    # Clean mask a bit (avoid anything that merges blobs!)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)

    # HoughCircles works better on a blurred image
    blur = cv2.GaussianBlur(mask, (9, 9), 2)

    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius
    )

    dets = []
    if circles is not None:
        circles = np.round(circles[0]).astype(int)
        for x, y, r in circles:
            area = float(np.pi * r * r)
            dets.append((int(x), int(y), int(r), area, 1.0))

    # Optional: sort biggest first
    dets.sort(key=lambda t: t[2], reverse=True)
    return dets, mask"""

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
        
def find_objects_in_image(img_bgr):
    warped, M, mask = find_arena_and_warp(img_bgr)
    if warped is None:
        return None, None

    orange_balls, omask = detect_balls_by_hsv(warped, lower=(5,120,120), upper=(25,255,255))
    white_balls, wmask   = detect_balls_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))

    return orange_balls, white_balls

# Used to generate pictures to see the results of the detection
# Not used in the actual bot, but can be useful for debugging and tuning parameters
if __name__ == "__main__":
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

        # Show results
        output_path = output_folder / img_path.name
        cv2.imwrite(str(output_path), vis)
