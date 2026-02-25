import cv2
import numpy as np

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

def detect_ball_by_hsv(warped_bgr, lower, upper, min_area=80, max_area=2000):
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8), iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None

    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area or area > max_area:
            continue

        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)

        if circularity < 0.6:
            continue

        (x, y), r = cv2.minEnclosingCircle(c)
        cand = (int(x), int(y), int(r), area, circularity)
        if best is None or area > best[3]:
            best = cand

    return best, mask

def px_to_world_cm(x_px, y_px, warp_w_px, warp_h_px, court_w_cm=120.0, court_h_cm=180.0, origin="top_left"):
    """
    origin:
      - "top_left":  (0,0) at top-left  -> y is distance from TOP wall
      - "bottom_left": (0,0) at bottom-left -> y is distance from BOTTOM wall
    Returns (x_cm, y_cm)
    """
    cm_per_px_x = court_w_cm / warp_w_px
    cm_per_px_y = court_h_cm / warp_h_px

    x_cm = x_px * cm_per_px_x
    y_cm_from_top = y_px * cm_per_px_y

    if origin == "top_left":
        return x_cm, y_cm_from_top
    elif origin == "bottom_left":
        y_cm_from_bottom = court_h_cm - y_cm_from_top
        return x_cm, y_cm_from_bottom
    else:
        raise ValueError("origin must be 'top_left' or 'bottom_left'")

if __name__ == "__main__":
    img = cv2.imread("../Images/20260225_091542.jpg")

    WARP_W, WARP_H = 800, 1200
    COURT_W_CM, COURT_H_CM = 120.0, 180.0

    warped, M, redmask = find_arena_and_warp(img, out_w=WARP_W, out_h=WARP_H)
    if warped is None:
        raise RuntimeError("Could not find arena")

    orange, omask = detect_ball_by_hsv(warped, lower=(5, 120, 120), upper=(25, 255, 255))
    white, wmask   = detect_ball_by_hsv(warped, lower=(0, 0, 180), upper=(180, 60, 255))

    def report(name, det):
        if det is None:
            print(f"{name}: not found")
            return
        x_px, y_px, r_px, area, circ = det

        # Distance from LEFT wall + distance from TOP wall
        """x_cm_left, y_cm_top = px_to_world_cm(
            x_px, y_px,
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
            origin="top_left"
        )"""

        # Distance from LEFT wall + distance from BOTTOM wall
        x_cm_left, y_cm_bottom = px_to_world_cm(
            x_px, y_px,
            warp_w_px=WARP_W, warp_h_px=WARP_H,
            court_w_cm=COURT_W_CM, court_h_cm=COURT_H_CM,
            origin="bottom_left"
        )

        print(f"{name}: px=(x={x_px}, y={y_px})")
        print(f"  from LEFT wall:   {x_cm_left:.2f} cm")
        #print(f"  from TOP wall:    {y_cm_top:.2f} cm")
        print(f"  from BOTTOM wall: {y_cm_bottom:.2f} cm")
        print(f"  radius: {r_px}px  area:{area:.1f}  circularity:{circ:.3f}")

    report("Orange", orange)
    report("White", white)