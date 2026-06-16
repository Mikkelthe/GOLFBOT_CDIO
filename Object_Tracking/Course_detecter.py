import cv2
import numpy as np
from matplotlib.image import imsave

class CourseDetector:
    def __init__(self):
        self.padding = 100

    @staticmethod
    def __hsv_mask_red(hsv):
        # red wraps hue -> two ranges
        lower1 = np.array([0, 20, 30])
        upper1 = np.array([30, 255, 255])
        lower2 = np.array([150, 40, 30])
        upper2 = np.array([180, 255, 255])
        return cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

    @staticmethod
    def __line_intersection(l1, l2):
        # lines in ax + by + c = 0 form
        a1, b1, c1 = l1
        a2, b2, c2 = l2
        d = a1*b2 - a2*b1
        if abs(d) < 1e-9:
            return None
        x = (b1*c2 - b2*c1) / d
        y = (c1*a2 - c2*a1) / d
        return np.array([x, y], dtype=np.float32)
    @staticmethod
    def __normalize_line_from_rho_theta(rho, theta):
        # Hough gives rho,theta for x*cos + y*sin = rho
        a = np.cos(theta)
        b = np.sin(theta)
        # => a*x + b*y - rho = 0
        return np.array([a, b, -rho], dtype=np.float32)

    def __find_box_corners_by_hough(self, img_bgr):
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        red = self.__hsv_mask_red(hsv)
        debug = red.copy()
        cv2.imwrite("debug_red.png", debug)
        
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
        L = self.__normalize_line_from_rho_theta(*left)
        R = self.__normalize_line_from_rho_theta(*right)
        T = self.__normalize_line_from_rho_theta(*top)
        B = self.__normalize_line_from_rho_theta(*bottom)

        # intersections: TL, TR, BR, BL
        tl = self.__line_intersection(T, L)
        tr = self.__line_intersection(T, R)
        br = self.__line_intersection(B, R)
        bl = self.__line_intersection(B, L)
        # padding the corners, to get a bit outside the arena aswell

        tr[0] += self.padding
        tr[1] += -self.padding
        tl[0] += -self.padding
        tl[1] += -self.padding
        br[0] += self.padding
        br[1] += self.padding
        bl[0] += -self.padding
        bl[1] += self.padding
        if any(p is None for p in [tl, tr, br, bl]):
            return None

        corners = np.stack([tl, tr, br, bl], axis=0)

        # sanity clamp (optional)
        corners[:, 0] = np.clip(corners[:, 0], 0, w-1)
        corners[:, 1] = np.clip(corners[:, 1], 0, h-1)

        return corners

    def __find_red_cross_contour(self, img_bgr):
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
        red_mask = self.hsv_mask_red(hsv)

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

        debug_mask = full_mask.copy()
        cv2.imwrite("debug_mask.png", debug_mask)


        return cross_contour, full_mask

    def find_red_cross_boxes(self, img_bgr):
        cross_contour, cross_mask = self.__find_red_cross_contour(img_bgr)
        if cross_contour is None:
            cv2.imwrite("debug_cross_mask.png", cross_mask)
            return None

        x, y, w, h = cv2.boundingRect(cross_contour)
        roi = cross_mask[y:y + h, x:x + w]

        ys, xs = np.where(roi > 0)
        if len(xs) == 0:
            return None

        cx = float(np.mean(xs))
        cy = float(np.mean(ys))

        # Find line segments in the red cross
        lines = cv2.HoughLinesP(
            roi,
            rho=1,
            theta=np.pi / 180,
            threshold=15,
            minLineLength=max(10, min(w, h) // 4),
            maxLineGap=20
        )

        if lines is None or len(lines) < 2:
            return None

        angles = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            dx = x2 - x1
            dy = y2 - y1

            if dx == 0 and dy == 0:
                continue

            angle = np.degrees(np.arctan2(dy, dx))

            if angle < 0:
                angle += 180

            angles.append(angle)

        if len(angles) < 2:
            return None

        angles = np.array(angles, dtype=np.float32)

        # Cluster angles into two dominant directions
        angle_features = np.column_stack([
            np.cos(np.deg2rad(2 * angles)),
            np.sin(np.deg2rad(2 * angles))
        ]).astype(np.float32)

        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            20,
            0.1
        )

        _, labels, _ = cv2.kmeans(
            angle_features,
            2,
            None,
            criteria,
            10,
            cv2.KMEANS_PP_CENTERS
        )

        labels = labels.flatten()

        direction_angles = []

        for cluster_id in [0, 1]:
            cluster_angles = angles[labels == cluster_id]

            if len(cluster_angles) == 0:
                continue

            mean_cos = np.mean(np.cos(np.deg2rad(2 * cluster_angles)))
            mean_sin = np.mean(np.sin(np.deg2rad(2 * cluster_angles)))

            mean_angle = 0.5 * np.degrees(np.arctan2(mean_sin, mean_cos))

            if mean_angle < 0:
                mean_angle += 180

            direction_angles.append(mean_angle)

        if len(direction_angles) < 2:
            return None

        boxes = []

        # This is the important change:
        # The two arm masks are NOT exclusive.
        # The center pixels may belong to both arms.
        half_thickness = max(8, min(w, h) // 7)

        for angle in direction_angles:
            angle_rad = np.deg2rad(angle)

            direction = np.array([
                np.cos(angle_rad),
                np.sin(angle_rad)
            ])

            arm_mask = np.zeros_like(roi)

            for px, py in zip(xs, ys):
                p = np.array([
                    px - cx,
                    py - cy
                ])

                # Perpendicular distance from pixel to the detected arm direction
                dist = abs(p[0] * direction[1] - p[1] * direction[0])

                if dist <= half_thickness:
                    arm_mask[py, px] = 255

            # Close small gaps in the arm
            kernel = np.ones((5, 5), np.uint8)
            arm_mask = cv2.morphologyEx(arm_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

            contours, _ = cv2.findContours(
                arm_mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if not contours:
                return None

            arm_contour = max(contours, key=cv2.contourArea)

            if cv2.contourArea(arm_contour) < 100:
                return None

            # Move back to original image coordinates
            arm_contour = arm_contour + np.array([[[x, y]]], dtype=np.int32)

            rect = cv2.minAreaRect(arm_contour)
            box = cv2.boxPoints(rect).astype(int)

            boxes.append(box)

        M = cv2.moments(cross_contour)
        center = None

        if M["m00"] != 0:
            center = (
                int(M["m10"] / M["m00"]),
                int(M["m01"] / M["m00"])
            )
        print("i am center " + str(center))
        return {
            "vertical_box": boxes[0],
            "horizontal_box": boxes[1],
            "center": center,
        }

    def find_arena(self, img, out_w, out_h):
        # corners must be TL,TR,BR,BL float32
        dst = np.array([[0,0],[out_w,0],[out_w,out_h],[0,out_h]], dtype=np.float32)
        corners = self.__find_box_corners_by_hough(img)

        if corners[0] is None:
            return img
        print(corners)
        M = cv2.getPerspectiveTransform(corners.astype(np.float32), dst)
        warped = cv2.warpPerspective(img, M, (out_w, out_h))
        return warped

