from dataclasses import dataclass
from math import hypot, isinf

from utils.point import Point

from robot_logic.route_planning.pathfinder import path_length, plan_smooth_path


@dataclass(frozen=True)
class Ball:
    name: str
    position: Point


def distance(point_a: Point, point_b: Point) -> float:
    return hypot(point_b.x - point_a.x, point_b.y - point_a.y)


def as_point(value) -> Point:
    if isinstance(value, Point):
        return value
    if hasattr(value, "position"):
        return as_point(value.position)
    if hasattr(value, "point"):
        return as_point(value.point)
    if hasattr(value, "x") and hasattr(value, "y"):
        return Point(value.x, value.y)
    return Point(value[0], value[1])


def _path_cost(start: Point, target: Point, obstacles) -> float:
    path = plan_smooth_path(start, target, obstacles=obstacles)
    return path_length(path) if path else float("inf")


def choose_best_next_ball(robot_position, balls, obstacles=None):
    if not balls:
        return None

    robot_point = as_point(robot_position)
    best_ball = None
    best_score = (float("inf"), float("inf"), float("inf"))

    for index, ball in enumerate(balls):
        ball_point = as_point(ball)
        cost = _path_cost(robot_point, ball_point, obstacles)
        if isinf(cost):
            continue

        score = (
            cost,
            distance(robot_point, ball_point),
            index,
        )
        if score < best_score:
            best_score = score
            best_ball = ball

    return best_ball


def plan_best_path(start, target, obstacles=None) -> list[Point]:
    return plan_smooth_path(as_point(start), as_point(target), obstacles=obstacles)


__all__ = [
    "Ball",
    "as_point",
    "choose_best_next_ball",
    "distance",
    "plan_best_path",
]
