from dataclasses import dataclass
from math import hypot
from typing import Iterable

from Object_Tracking.Object_Tracking import px_to_world_cm
from robot_logic.navigation_config import (
    CROSS_FALLBACK_RADIUS_CM,
    CROSS_SAFETY_MARGIN_CM,
    WARP_H,
    WARP_W,
)
from robot_logic.route_planning.route_planner import Point, distance


@dataclass(frozen=True)
class AxisAlignedBox:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def contains(self, point: Point, inflation_cm: float = 0.0) -> bool:
        return (
            self.min_x - inflation_cm <= point.x <= self.max_x + inflation_cm
            and self.min_y - inflation_cm <= point.y <= self.max_y + inflation_cm
        )


@dataclass(frozen=True)
class CrossObstacle:
    arms: tuple[AxisAlignedBox, ...] = ()
    fallback_center: Point | None = None
    fallback_radius_cm: float = CROSS_FALLBACK_RADIUS_CM
    safety_margin_cm: float = CROSS_SAFETY_MARGIN_CM

    def contains(self, point: Point) -> bool:
        if self.arms:
            return any(arm.contains(point, self.safety_margin_cm) for arm in self.arms)

        if self.fallback_center is None:
            return False

        return distance(point, self.fallback_center) <= self.fallback_radius_cm


def _box_from_world_points(points: Iterable[Point]) -> AxisAlignedBox | None:
    point_list = list(points)
    if not point_list:
        return None

    return AxisAlignedBox(
        min(point.x for point in point_list),
        max(point.x for point in point_list),
        min(point.y for point in point_list),
        max(point.y for point in point_list),
    )


def _pixel_box_to_world_box(box, warp_w_px: int, warp_h_px: int) -> AxisAlignedBox | None:
    world_points = []

    for corner in box:
        x_px, y_px = float(corner[0]), float(corner[1])
        x_cm, y_cm = px_to_world_cm(x_px, y_px, warp_w_px, warp_h_px)
        world_points.append(Point(x_cm, y_cm))

    return _box_from_world_points(world_points)


def _pixel_center_to_world(center, warp_w_px: int, warp_h_px: int) -> Point | None:
    if center is None:
        return None

    x_cm, y_cm = px_to_world_cm(center[0], center[1], warp_w_px, warp_h_px)
    return Point(x_cm, y_cm)


def build_cross_obstacle(
    cross_position,
    warp_w_px: int = WARP_W,
    warp_h_px: int = WARP_H,
    safety_margin_cm: float = CROSS_SAFETY_MARGIN_CM,
    fallback_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
) -> CrossObstacle | None:
    if cross_position is None:
        return None

    if isinstance(cross_position, CrossObstacle):
        return cross_position

    if isinstance(cross_position, Point):
        return CrossObstacle(
            fallback_center=cross_position,
            fallback_radius_cm=fallback_radius_cm,
            safety_margin_cm=safety_margin_cm,
        )

    if isinstance(cross_position, dict):
        arms = []

        for key in ("vertical_box", "horizontal_box"):
            if key not in cross_position or cross_position[key] is None:
                continue

            arm = _pixel_box_to_world_box(cross_position[key], warp_w_px, warp_h_px)
            if arm is not None:
                arms.append(arm)

        center = _pixel_center_to_world(cross_position.get("center"), warp_w_px, warp_h_px)
        if arms or center is not None:
            return CrossObstacle(
                arms=tuple(arms),
                fallback_center=center,
                fallback_radius_cm=fallback_radius_cm,
                safety_margin_cm=safety_margin_cm,
            )

        return None

    try:
        if len(cross_position) >= 2:
            return CrossObstacle(
                fallback_center=Point(float(cross_position[0]), float(cross_position[1])),
                fallback_radius_cm=fallback_radius_cm,
                safety_margin_cm=safety_margin_cm,
            )
    except TypeError:
        return None

    return None


def straight_line_crosses_obstacle(
    start: Point,
    end: Point,
    obstacle: CrossObstacle | None,
    sample_step_cm: float = 2.5,
) -> bool:
    if obstacle is None:
        return False

    segment_length = hypot(end.x - start.x, end.y - start.y)
    if segment_length == 0:
        return obstacle.contains(start)

    samples = max(1, int(segment_length / sample_step_cm))
    for index in range(samples + 1):
        t = index / samples
        point = Point(
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
        )
        if obstacle.contains(point):
            return True

    return False
