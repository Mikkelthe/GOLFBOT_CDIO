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
    br = line_intersection(B, R)
    bl = line_intersection(B, L)

    if any(p is None for p in [tl, tr, br, bl]):
        return None, red, edges

    corners = np.stack([tl, tr, br, bl], axis=0)

    # sanity clamp (optional)
    corners[:, 0] = np.clip(corners[:, 0], 0, w-1)
    corners[:, 1] = np.clip(corners[:, 1], 0, h-1)

    return corners

def Find_Arena(img, out_w, out_h):
    # corners must be TL,TR,BR,BL float32
    dst = np.array([[0,0],[out_w,0],[out_w,out_h],[0,out_h]], dtype=np.float32)
    corners = find_box_corners_by_hough(img)
    if corners is None:
        return None, None
    M = cv2.getPerspectiveTransform(corners.astype(np.float32), dst)
    warped = cv2.warpPerspective(img, M, (out_w, out_h))
    return warped