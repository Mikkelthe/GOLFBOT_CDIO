import cv2
import numpy as np

def hsv_mask_red(hsv):
    # red wraps hue -> two ranges
    lower1 = np.array([0, 80, 60])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([170, 80, 60])
    upper2 = np.array([180, 255, 255])
    return cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

def line_intersection(l1, l2):
    # lines in ax + by + c = 0 form
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    d = a1*b2 - a2*b1
    if abs(d) < 1e-9:
        return None
    x = (b1*c2 - b2*c1) / d
    y = (c1*a2 - c2*a1) / d
    return np.array([x, y], dtype=np.float32)

def normalize_line_from_rho_theta(rho, theta):
    # Hough gives rho,theta for x*cos + y*sin = rho
    a = np.cos(theta)
    b = np.sin(theta)
    # => a*x + b*y - rho = 0
    return np.array([a, b, -rho], dtype=np.float32)

def find_box_corners_by_hough(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    red = hsv_mask_red(hsv)

    
    k = np.ones((7,7), np.uint8)
    red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, k, iterations=2)
    red = cv2.morphologyEx(red, cv2.MORPH_OPEN,  k, iterations=1)

    edges = cv2.Canny(red, 50, 150)

    # Finds red lines in image
    lines = cv2.HoughLines(edges, 1, np.pi/180, 160)
    if lines is None:
        return None, red, edges

    h, w = red.shape[:2]

    # classify lines into near-vertical and near-horizontal
    vertical = []
    horizontal = []
    for (rho, theta) in lines[:, 0]:
        # normalize angle to [0, pi)
        t = theta
        # vertical lines have theta near 0 or pi (normal points left/right)
        # horizontal lines have theta near pi/2 (normal points up/down)
        if t < np.pi/4 or t > 3*np.pi/4:
            vertical.append((rho, theta))
        else:
            horizontal.append((rho, theta))

    if len(vertical) < 2 or len(horizontal) < 2:
        return None, red, edges

    # pick left/right by rho after converting to line form and evaluating x at mid-y
    def x_at_y(rho, theta, y):
        a = np.cos(theta); b = np.sin(theta)
        # a*x + b*y = rho => x = (rho - b*y)/a
        if abs(a) < 1e-6:
            return None
        return (rho - b*y) / a

    ymid = h / 2.0
    vxs = []
    for rho, theta in vertical:
        x = x_at_y(rho, theta, ymid)
        if x is not None:
            vxs.append((x, rho, theta))
    vxs.sort(key=lambda t: t[0])
    left = vxs[0][1], vxs[0][2]
    right = vxs[-1][1], vxs[-1][2]

    # pick top/bottom by y at mid-x
    def y_at_x(rho, theta, x):
        a = np.cos(theta); b = np.sin(theta)
        # a*x + b*y = rho => y = (rho - a*x)/b
        if abs(b) < 1e-6:
            return None
        return (rho - a*x) / b

    xmid = w / 2.0
    hys = []
    for rho, theta in horizontal:
        y = y_at_x(rho, theta, xmid)
        if y is not None:
            hys.append((y, rho, theta))
    hys.sort(key=lambda t: t[0])
    top = hys[0][1], hys[0][2]
    bottom = hys[-1][1], hys[-1][2]

    # convert to ax + by + c = 0
    L = normalize_line_from_rho_theta(*left)
    R = normalize_line_from_rho_theta(*right)
    T = normalize_line_from_rho_theta(*top)
    B = normalize_line_from_rho_theta(*bottom)

    # intersections: TL, TR, BR, BL
    tl = line_intersection(T, L)
    tr = line_intersection(T, R)
    print(tr)
    print(tl)
    br = line_intersection(B, R)
    bl = line_intersection(B, L)
    tr[0] += 50
    tr[1] += -50
    print(tr)
    tl[0] += -50
    tl[1] += -50
    br[0] += 50
    br[1] += 50
    bl[0] += -50
    bl[1] += 50
    if any(p is None for p in [tl, tr, br, bl]):
        return None, red, edges

    corners = np.stack([tl, tr, br, bl], axis=0)

    # sanity clamp (optional)
    corners[:, 0] = np.clip(corners[:, 0], 0, w-1)
    corners[:, 1] = np.clip(corners[:, 1], 0, h-1)

    return corners


def touches_border(contour, img_w, img_h, margin=5):
    x, y, w, h = cv2.boundingRect(contour)
    return (
        x <= margin or
        y <= margin or
        x + w >= img_w - margin or
        y + h >= img_h - margin
    )


def find_red_cross_contour(img_bgr):
    img_h, img_w = img_bgr.shape[:2]

    # Middle search area size
    roi_w = 400
    roi_h = 300

    # Calculate centered ROI
    x0 = max(0, img_w // 2 - roi_w // 2)
    y0 = max(0, img_h // 2 - roi_h // 2)
    x1 = min(img_w, x0 + roi_w)
    y1 = min(img_h, y0 + roi_h)

    # Crop image to only middle area
    roi_bgr = img_bgr[y0:y1, x0:x1]
    #debug = roi_bgr.copy()
    #cv2.imwrite("debug_middle_roi.png", debug)

    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    red_mask = hsv_mask_red(hsv)

    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # Connected components
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(red_mask, connectivity=8)

    h, w = red_mask.shape
    best_label = None
    best_area = 0

    for label in range(1, num_labels):  # skip background
        x = stats[label, cv2.CC_STAT_LEFT]
        y = stats[label, cv2.CC_STAT_TOP]
        ww = stats[label, cv2.CC_STAT_WIDTH]
        hh = stats[label, cv2.CC_STAT_HEIGHT]
        area = stats[label, cv2.CC_STAT_AREA]

        # ignore tiny blobs
        if area < 300:
            continue

        # ignore anything touching image border
        if x == 0 or y == 0 or (x + ww) >= w or (y + hh) >= h:
            continue

        if area > best_area:
            best_area = area
            best_label = label

    if best_label is None:
        return None, red_mask

    cross_mask = np.zeros_like(red_mask)
    cross_mask[labels == best_label] = 255

    contours, _ = cv2.findContours(cross_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, red_mask

    cross_contour = max(contours, key=cv2.contourArea)
    cross_contour = cross_contour + np.array([[[x0, y0]]], dtype=np.int32)


    full_mask = np.zeros((img_h, img_w), dtype=np.uint8)
    full_mask[y0:y1, x0:x1] = cross_mask

    #debug_mask = full_mask.copy()
    #cv2.imwrite("debug_mask.png", debug_mask)


    return cross_contour, full_mask

def find_red_cross_boxes(img_bgr):
    cross_contour, cross_mask = find_red_cross_contour(img_bgr)
    if cross_contour is None:
        cv2.imwrite("debug_cross_mask.png", cross_mask)
        return None

    x, y, w, h = cv2.boundingRect(cross_contour)
    roi = cross_mask[y:y+h, x:x+w]

    ys, xs = np.where(roi > 0)
    if len(xs) == 0:
        return None

    cx = int(np.mean(xs))
    cy = int(np.mean(ys))

    band = max(10, min(w, h) // 5)

    vertical_mask = np.zeros_like(roi)
    horizontal_mask = np.zeros_like(roi)

    x1 = max(0, cx - band)
    x2 = min(roi.shape[1], cx + band)
    y1 = max(0, cy - band)
    y2 = min(roi.shape[0], cy + band)

    vertical_mask[:, x1:x2] = roi[:, x1:x2]
    horizontal_mask[y1:y2, :] = roi[y1:y2, :]

    v_contours, _ = cv2.findContours(vertical_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h_contours, _ = cv2.findContours(horizontal_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not v_contours or not h_contours:
        return None

    v_contour = max(v_contours, key=cv2.contourArea)
    h_contour = max(h_contours, key=cv2.contourArea)

    v_contour = v_contour + np.array([[[x, y]]], dtype=np.int32)
    h_contour = h_contour + np.array([[[x, y]]], dtype=np.int32)

    v_rect = cv2.minAreaRect(v_contour)
    h_rect = cv2.minAreaRect(h_contour)

    v_box = cv2.boxPoints(v_rect).astype(int)
    h_box = cv2.boxPoints(h_rect).astype(int)

    M = cv2.moments(cross_contour)
    center = None
    if M["m00"] != 0:
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

    return {
        "vertical_box": v_box,
        "horizontal_box": h_box,
        "center": center,
    }
    
def find_red_cross_center(img_bgr):
    cross_contour, _ = find_red_cross_contour(img_bgr)
    if cross_contour is None:
        return None

    M = cv2.moments(cross_contour)
    if M["m00"] == 0:
        return None

    x = int(M["m10"] / M["m00"])
    y = int(M["m01"] / M["m00"])
    return (x, y)
"""
def find_red_cross_center(img_bgr):
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    red_mask = hsv_mask_red(hsv)

    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    if len(contours) < 2:
        return None

    cross_contour = contours[1]

    M = cv2.moments(cross_contour)
    if M["m00"] == 0:
        return None

    x = int(M["m10"] / M["m00"])
    y = int(M["m01"] / M["m00"])

    return (x, y)
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    red_mask = hsv_mask_red(hsv)

    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Largest red contour is usually the border, second largest should be the cross
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    if len(contours) < 2:
        return None

    cross_contour = contours[1]

    # Bounding box around the whole cross area
    x, y, w, h = cv2.boundingRect(cross_contour)

    # Crop mask to only cross area
    roi = red_mask[y:y+h, x:x+w]

    # Connected pixels of the cross
    ys, xs = np.where(roi > 0)
    if len(xs) == 0:
        return None

    # Estimate center of cross within ROI
    cx = int(np.mean(xs))
    cy = int(np.mean(ys))

    # Split into vertical and horizontal parts using bands around center
    band = max(8, min(w, h) // 6)

    vertical_mask = np.zeros_like(roi)
    horizontal_mask = np.zeros_like(roi)

    vertical_mask[:, max(0, cx - band):min(roi.shape[1], cx + band)] = roi[:, max(0, cx - band):min(roi.shape[1], cx + band)]
    horizontal_mask[max(0, cy - band):min(roi.shape[0], cy + band), :] = roi[max(0, cy - band):min(roi.shape[0], cy + band), :]

    # Find contour for vertical bar
    v_contours, _ = cv2.findContours(vertical_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h_contours, _ = cv2.findContours(horizontal_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not v_contours or not h_contours:
        return None

    v_contour = max(v_contours, key=cv2.contourArea)
    h_contour = max(h_contours, key=cv2.contourArea)

    # Shift contours back into full-image coordinates
    v_contour = v_contour + np.array([[[x, y]]], dtype=np.int32)
    h_contour = h_contour + np.array([[[x, y]]], dtype=np.int32)

    # Rotated rectangles
    v_rect = cv2.minAreaRect(v_contour)
    h_rect = cv2.minAreaRect(h_contour)

    v_box = cv2.boxPoints(v_rect).astype(int)
    h_box = cv2.boxPoints(h_rect).astype(int)

    return {
        "vertical_box": v_box,
        "horizontal_box": h_box,
        "center": (x + cx, y + cy)
    }
"""

def find_arena(img, out_w, out_h):
    # corners must be TL,TR,BR,BL float32
    dst = np.array([[0,0],[out_w,0],[out_w,out_h],[0,out_h]], dtype=np.float32)
    corners = find_box_corners_by_hough(img)
    if corners is None:
        return None, None
    M = cv2.getPerspectiveTransform(corners.astype(np.float32), dst)
    warped = cv2.warpPerspective(img, M, (out_w, out_h))
    return warped

