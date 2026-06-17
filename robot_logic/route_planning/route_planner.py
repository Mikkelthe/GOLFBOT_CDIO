from dataclasses import dataclass
from math import hypot, isinf

from utils.point import Point

from robot_logic.route_planning.pathfinder import pathfinder


@dataclass(frozen=True)
class Ball:
    name: str
    position: Point

class RoutePlanner:
    def __init__(self):
        self.pathfinder = pathfinder()
    @staticmethod

    def __distance(point_a: Point, point_b: Point) -> float:
        return hypot(point_b.x - point_a.x, point_b.y - point_a.y)


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


    def __path_cost(self, start: Point, target: Point, obstacles) -> float:
        path = self.pathfinder.plan_smooth_path(start, target, obstacles=obstacles)
        return self.pathfinder.path_length(path) if path else float("inf")


    def choose_best_next_ball(self, robot_position, balls, obstacles=None):
        if not balls:
            return None

        robot_point = self.__as_point(robot_position)
        best_ball = None
        best_score = (float("inf"), float("inf"), float("inf"))

        for index, ball in enumerate(balls):
            ball_point = self.__as_point(ball)
            cost = self.__path_cost(robot_point, ball_point, obstacles)
            if isinf(cost):
                continue

            score = (
                cost,
                self.__distance(robot_point, ball_point),
                index,
            )
            if score < best_score:
                best_score = score
                best_ball = ball

        return best_ball


    def plan_best_path(self, start, target, obstacles=None) -> list[Point]:
        return self.pathfinder.plan_smooth_path(self.__as_point(start), self.__as_point(target), obstacles=obstacles)



