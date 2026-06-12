from dataclasses import dataclass
from math import atan2, degrees

import cv2
import numpy as np

from Object_Tracking.Course_detecter import find_box_corners_by_hough
from Object_Tracking.Object_Tracking import px_to_world_cm
from robot_logic.navigation_config import (
    APPLY_MARKER_PARALLAX_CORRECTION,
    CAMERA_HEIGHT_CM,
    MARKER_TO_ROBOT_HEADING_OFFSET_DEGREES,
    ROBOT_MARKER_HEIGHT_CM,
    WARP_H,
    WARP_W,
)
from robot_logic.route_planning.route_planner import Point


DEFAULT_MARKER_ID = 0
DEFAULT_WARP_W = WARP_W
DEFAULT_WARP_H = WARP_H


@dataclass(frozen=True)
class RobotPose:
    position: Point
    heading_degrees: float
    marker_id: int


def _marker_detection_images(image):
    if len(image.shape) == 2:
        gray_image = image
    else:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    yield gray_image
    yield cv2.equalizeHist(gray_image)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    yield clahe.apply(gray_image)

    # The new camera captures tint the white marker border blue; direct grayscale
    # detection misses those frames, but local thresholding restores the border.
    yield cv2.adaptiveThreshold(
        gray_image,
        255,
        cv2.ADAPTIVE_THRESH_MEAN_C,
        cv2.THRESH_BINARY,
        31,
        5,
    )
    yield cv2.adaptiveThreshold(
        gray_image,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        5,
    )


def detect_aruco_marker_corners(
    image,
    marker_id: int = DEFAULT_MARKER_ID,
) -> np.ndarray | None:
    if image is None or not hasattr(cv2, "aruco"):
        return None

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary)

    for detection_image in _marker_detection_images(image):
        detected_corners, detected_ids, _ = detector.detectMarkers(detection_image)

        if detected_ids is None:
            continue

        for corners, detected_id in zip(detected_corners, detected_ids.flatten()):
            if int(detected_id) == marker_id:
                return corners[0]

    return None


def marker_center_to_ground_pixel(
    center_x: float,
    center_y: float,
    warp_w_px: int,
    warp_h_px: int,
) -> tuple[float, float]:
    if not APPLY_MARKER_PARALLAX_CORRECTION:
        return center_x, center_y

    scale = (CAMERA_HEIGHT_CM - ROBOT_MARKER_HEIGHT_CM) / CAMERA_HEIGHT_CM
    field_center_x = warp_w_px / 2.0
    field_center_y = warp_h_px / 2.0
    ground_x = field_center_x + (center_x - field_center_x) * scale
    ground_y = field_center_y + (center_y - field_center_y) * scale
    return ground_x, ground_y


def detect_robot_pose(
    raw_image,
    marker_id: int = DEFAULT_MARKER_ID,
    warp_w_px: int = DEFAULT_WARP_W,
    warp_h_px: int = DEFAULT_WARP_H,
) -> RobotPose | None:
    if raw_image is None or not hasattr(cv2, "aruco"):
        return None

    marker_corners = detect_aruco_marker_corners(raw_image, marker_id)
    if marker_corners is None:
        return None

    arena_corners = find_box_corners_by_hough(raw_image)
    if isinstance(arena_corners, tuple):
        arena_corners = arena_corners[0]

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
    ground_x, ground_y = marker_center_to_ground_pixel(center_x, center_y, warp_w_px, warp_h_px)
    position_x_cm, position_y_cm = px_to_world_cm(ground_x, ground_y, warp_w_px, warp_h_px)

    top_mid_x = float((warped_marker_corners[0, 0] + warped_marker_corners[1, 0]) / 2.0)
    top_mid_y = float((warped_marker_corners[0, 1] + warped_marker_corners[1, 1]) / 2.0)

    dx_world = top_mid_x - center_x
    dy_world = -(top_mid_y - center_y)
    marker_heading_degrees = degrees(atan2(dy_world, dx_world))
    # Apply any measured orientation offset between the marker and robot front.
    heading_degrees = marker_heading_degrees + MARKER_TO_ROBOT_HEADING_OFFSET_DEGREES

    return RobotPose(
        position=Point(position_x_cm, position_y_cm),
        heading_degrees=heading_degrees,
        marker_id=marker_id,
    )
