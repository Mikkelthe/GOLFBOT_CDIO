from dataclasses import dataclass
from math import cos, radians, sin

from Navigation.Navigation import find_bot
from Object_Tracking.Course_detecter import find_arena
from Object_Tracking.Object_Tracking import cm_to_px, px_to_world_cm
from robot_logic.navigation_config import (
    CAMERA_HEIGHT_CM,
    MARKER_TO_ROBOT_CENTER_OFFSET_CM,
    ROBOT_MARKER_HEIGHT_CM,
    WARP_H,
    WARP_W,
)
from robot_logic.route_planning.route_planner import Point


DEFAULT_MARKER_ID = 0


@dataclass(frozen=True)
class RobotPose:
    position: Point
    heading_degrees: float
    marker_id: int = DEFAULT_MARKER_ID


def detect_robot_pose(
    raw_image,
    marker_id: int = DEFAULT_MARKER_ID,
    warp_w_px: int = WARP_W,
    warp_h_px: int = WARP_H,
) -> RobotPose | None:
    if raw_image is None:
        return None

    warped = find_arena(raw_image, out_w=warp_w_px, out_h=warp_h_px)
    if isinstance(warped, tuple):
        warped = warped[0]

    if warped is None or not hasattr(warped, "shape"):
        return None

    marker, heading = find_bot(warped)
    if marker is None or heading is None:
        return None

    scale = (CAMERA_HEIGHT_CM - ROBOT_MARKER_HEIGHT_CM) / CAMERA_HEIGHT_CM
    center_x, center_y = warp_w_px / 2.0, warp_h_px / 2.0
    offset_px = cm_to_px(
        MARKER_TO_ROBOT_CENTER_OFFSET_CM,
        warp_w_px=warp_w_px,
        warp_h_px=warp_h_px,
    )
    angle = radians(heading)

    ground_x = center_x + (marker.x - center_x) * scale + offset_px * cos(angle)
    ground_y = center_y + (marker.y - center_y) * scale + offset_px * sin(angle)
    position_x_cm, position_y_cm = px_to_world_cm(ground_x, ground_y, warp_w_px, warp_h_px)
    return RobotPose(Point(position_x_cm, position_y_cm), heading, marker_id)
