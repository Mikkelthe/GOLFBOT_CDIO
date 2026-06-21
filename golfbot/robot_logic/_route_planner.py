from dataclasses import dataclass
from collections.abc import Mapping
from math import hypot, isinf

from utils import Point, Conversion
from ._pathfinder import Pathfinder


@dataclass(frozen=True)
class Ball:
    name: str
    position: Point

class RoutePlanner:
    def __init__(self):
        self.pathfinder = Pathfinder()
        self.converter = Conversion()
    @staticmethod

    def __distance(point_a: Point, point_b: Point) -> float:
        return hypot(point_b.x - point_a.x, point_b.y - point_a.y)


    @staticmethod
    def __is_corner_like(value) -> bool:
        if isinstance(value, Point):
            return True
        if hasattr(value, "x") and hasattr(value, "y"):
            return True

        try:
            _ = value[0]
            _ = value[1]
            return len(value) == 2
        except (TypeError, IndexError, KeyError, ValueError):
            return False


    def __as_point(self, value) -> Point:
        if isinstance(value, Point):
            return value
        if hasattr(value, "position"):
            return self.__as_point(value.position)
        if hasattr(value, "point"):
            return self.__as_point(value.point)
        if hasattr(value, "x") and hasattr(value, "y"):
            return Point(value.x, value.y)
        return Point(value[0], value[1])


    def __to_world_cm(self, point: Point) -> Point:
        x_cm, y_cm = self.converter.px_to_world_cm(point.x, point.y)
        return Point(x_cm, y_cm)


    def __to_pixel(self, point: Point) -> Point:
        x_px, y_px = self.converter.world_cm_to_px(point.x, point.y)
        return Point(x_px, y_px)


    def __obstacle_to_world_cm(self, obstacle):
        return tuple(self.__to_world_cm(self.__as_point(corner)) for corner in obstacle)


    def __obstacles_to_world_cm(self, obstacles):
        if obstacles is None:
            return None

        if isinstance(obstacles, Mapping):
            if "vertical_box" in obstacles or "horizontal_box" in obstacles:
                return {
                    key: self.__obstacle_to_world_cm(obstacles[key])
                    for key in ("vertical_box", "horizontal_box")
                    if key in obstacles and obstacles[key] is not None
                }

            return {
                key: self.__obstacle_to_world_cm(obstacle)
                for key, obstacle in obstacles.items()
                if obstacle is not None
            }

        obstacle_items = tuple(obstacles)
        if (
            len(obstacle_items) == 4
            and all(self.__is_corner_like(corner) for corner in obstacle_items)
        ):
            return (self.__obstacle_to_world_cm(obstacle_items),)

        return tuple(self.__obstacle_to_world_cm(obstacle) for obstacle in obstacle_items)


    def __path_cost(self, start: Point, target: Point, obstacles) -> float:
        path = self.pathfinder.plan_smooth_path(start, target, obstacles=obstacles)
        return self.pathfinder.path_length(path) if path else float("inf")


    def choose_best_next_ball(self, robot_position, balls, obstacles=None):
        if not balls:
            return None

        robot_point = self.__to_world_cm(self.__as_point(robot_position))
        world_obstacles = self.__obstacles_to_world_cm(obstacles)
        candidates = []

        for index, ball in enumerate(balls):
            ball_point = self.__to_world_cm(self.__as_point(ball))
            candidates.append((
                self.__distance(robot_point, ball_point),
                index,
                ball,
                ball_point,
            ))

        candidates.sort(key=lambda candidate: (candidate[0], candidate[1]))
        best_ball = None
        best_score = (float("inf"), float("inf"), float("inf"))

        for straight_distance, index, ball, ball_point in candidates:
            if straight_distance > best_score[0]:
                break

            cost = self.__path_cost(robot_point, ball_point, world_obstacles)
            if isinf(cost):
                continue

            score = (
                cost,
                straight_distance,
                index,
            )
            if score < best_score:
                best_score = score
                best_ball = ball

        return best_ball


    def plan_best_path(self, start, target, obstacles=None) -> list[Point]:
        start_point = self.__as_point(start)
        target_point = self.__as_point(target)
        world_path = self.pathfinder.plan_smooth_path(
            self.__to_world_cm(start_point),
            self.__to_world_cm(target_point),
            obstacles=self.__obstacles_to_world_cm(obstacles),
        )
        pixel_path = [self.__to_pixel(point) for point in world_path]

        if pixel_path:
            pixel_path[0] = start_point
            pixel_path[-1] = target_point

        return pixel_path

