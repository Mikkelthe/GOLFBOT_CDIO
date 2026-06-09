from dataclasses import dataclass
from math import atan2, degrees

import cv2
import numpy as np

from Object_Tracking.Course_detecter import find_box_corners_by_hough
from Object_Tracking.Object_Tracking import px_to_world_cm
from robot_logic.route_planning.route_planner import Point


DEFAULT_MARKER_ID = 0
DEFAULT_WARP_W = 800
DEFAULT_WARP_H = 1200


@dataclass(frozen=True)
class RobotPose:
    position: Point
    heading_degrees: float
    marker_id: int


def detect_robot_pose(
    raw_image,
    marker_id: int = DEFAULT_MARKER_ID,
    warp_w_px: int = DEFAULT_WARP_W,
    warp_h_px: int = DEFAULT_WARP_H,
) -> RobotPose | None:
    if raw_image is None or not hasattr(cv2, "aruco"):
        return None

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary)
    gray_image = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
    detected_corners, detected_ids, _ = detector.detectMarkers(gray_image)

    if detected_ids is None:
        return None

    marker_corners = None
    for corners, detected_id in zip(detected_corners, detected_ids.flatten()):
        if int(detected_id) == marker_id:
            marker_corners = corners[0]
            break

    if marker_corners is None:
        return None

    arena_corners = find_box_corners_by_hough(raw_image)
    if arena_corners is None:
        return None

    destination_corners = np.array(
        [[0, 0], [warp_w_px, 0], [warp_w_px, warp_h_px], [0, warp_h_px]],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(arena_corners.astype(np.float32), destination_corners)

    warped_marker_corners = cv2.perspectiveTransform(
        marker_corners.reshape(-1, 1, 2).astype(np.float32),
        homography,
    ).reshape(-1, 2)

    center_x = float(np.mean(warped_marker_corners[:, 0]))
    center_y = float(np.mean(warped_marker_corners[:, 1]))
    position_x_cm, position_y_cm = px_to_world_cm(center_x, center_y, warp_w_px, warp_h_px)

    top_mid_x = float((warped_marker_corners[0, 0] + warped_marker_corners[1, 0]) / 2.0)
    top_mid_y = float((warped_marker_corners[0, 1] + warped_marker_corners[1, 1]) / 2.0)

    dx_world = top_mid_x - center_x
    dy_world = -(top_mid_y - center_y)
    heading_degrees = degrees(atan2(dy_world, dx_world))

    return RobotPose(
        position=Point(position_x_cm, position_y_cm),
        heading_degrees=heading_degrees,
        marker_id=marker_id,
    )
