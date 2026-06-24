from dataclasses import dataclass
from collections.abc import Mapping
from math import hypot, isinf
from utils import Point, Conversion
from utils.settings import court_settings
from ._pathfinder import Pathfinder
from ._navigation_config import ROBOT_RADIUS_CM


@dataclass(frozen=True)
class Ball:
    name: str
    position: Point

class RoutePlanner:
    def __init__(self):
        self.pathfinder = Pathfinder()
        self.converter = Conversion()

    @staticmethod
    def __robot_wall_clearance_px() -> tuple[float, float]:
        court_width_px = court_settings.image_width - 2 * court_settings.padding
        court_height_px = court_settings.image_height - 2 * court_settings.padding
        extra_clearance_px = court_settings.wall_clearance_extra_px

        return (
            (ROBOT_RADIUS_CM * court_width_px / court_settings.court_width) + extra_clearance_px,
            (ROBOT_RADIUS_CM * court_height_px / court_settings.court_height) + extra_clearance_px,
        )

    @staticmethod
    def __safe_target_bounds_px() -> tuple[float, float, float, float]:
        clearance_x_px, clearance_y_px = RoutePlanner.__robot_wall_clearance_px()
        wall_inset_px = court_settings.padding + court_settings.wall_thickness

        return (
            wall_inset_px + clearance_x_px,
            court_settings.image_width - wall_inset_px - clearance_x_px,
            wall_inset_px + clearance_y_px,
            court_settings.image_height - wall_inset_px - clearance_y_px,
        )

    @staticmethod
    def __project_target_to_safe_bounds(point: Point) -> Point:
        min_x, max_x, min_y, max_y = RoutePlanner.__safe_target_bounds_px()
        return Point(
            min(max(point.x, min_x), max_x),
            min(max(point.y, min_y), max_y),
        )

    @staticmethod
    def __points_equal(point_a: Point, point_b: Point) -> bool:
        return point_a.x == point_b.x and point_a.y == point_b.y

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


    @staticmethod
    def __is_obstacle_like(value) -> bool:
        try:
            corners = tuple(value)
        except TypeError:
            return False

        return (
            len(corners) == 4
            and all(RoutePlanner.__is_corner_like(corner) for corner in corners)
        )


    @staticmethod
    def __field_boundaries() -> tuple[tuple[Point, Point, Point, Point], ...]:
        width = court_settings.image_width
        height = court_settings.image_height
        padding = court_settings.padding
        wall_thickness = court_settings.wall_thickness

        return (
            # top
            (
                Point(0, 0),
                Point(width, 0),
                Point(width, padding + wall_thickness),
                Point(0, padding + wall_thickness)
            ),
            # left
            (
                Point(0, 0),
                Point(padding + wall_thickness, 0),
                Point(padding + wall_thickness, height),
                Point(0, height)
            ),
            # right
            (
                Point(width - padding + wall_thickness, 0),
                Point(width, 0),
                Point(width, height),
                Point(width - padding + wall_thickness, height),
            ),
            # bottom
            (
                Point(0, height - padding + wall_thickness),
                Point(width, height - padding + wall_thickness),
                Point(width, height),
                Point(0, height),
            ),
        )


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


    def __obstacle_to_world_cm(self, obstacle: dict):
        return tuple(self.__to_world_cm(self.__as_point(corner)) for corner in obstacle)


    def __obstacles_to_world_cm(self, obstacles: dict):
        # walls are always included even when no cross is detected
        obstacle_polygons = list(self.__field_boundaries())

        if isinstance(obstacles, Mapping):
            # ignore metadata such as the cross center
            obstacle_polygons.extend(
                obstacle
                for obstacle in obstacles.values()
                if self.__is_obstacle_like(obstacle)
            )
            return tuple(self.__obstacle_to_world_cm(obstacle) for obstacle in obstacle_polygons)

        if obstacles is None:
            return tuple(self.__obstacle_to_world_cm(obstacle) for obstacle in obstacle_polygons)

        try:
            obstacle_items = tuple(obstacles)
        except TypeError:
            return tuple(self.__obstacle_to_world_cm(obstacle) for obstacle in obstacle_polygons)

        if (
            len(obstacle_items) == 4
            and all(self.__is_corner_like(corner) for corner in obstacle_items)
        ):
            obstacle_polygons.append(obstacle_items)
        else:
            obstacle_polygons.extend(
                obstacle
                for obstacle in obstacle_items
                if self.__is_obstacle_like(obstacle)
            )

        return tuple(self.__obstacle_to_world_cm(obstacle) for obstacle in obstacle_polygons)


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
            ball_pixel = self.__as_point(ball)
            drive_target_pixel = self.__project_target_to_safe_bounds(ball_pixel)
            ball_point = self.__to_world_cm(drive_target_pixel)
            candidates.append((
                self.__distance(robot_point, ball_point),
                index,
                ball,
                ball_point,
            ))

        # try nearby balls first so path searches can stop early
        candidates.sort(key=lambda candidate: (candidate[0], candidate[1]))
        best_ball = None
        best_score = (float("inf"), float("inf"), float("inf"))

        for straight_distance, index, ball, ball_point in candidates:
            # no path can beat its own straight-line distance
            if straight_distance > best_score[0]:
                break

            cost = self.__path_cost(robot_point, ball_point, world_obstacles)
            # skip balls that cannot be reached safely
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
        safe_start_point = self.__project_target_to_safe_bounds(start_point)
        drive_target_point = self.__project_target_to_safe_bounds(target_point)

        # when the robot starts outside the safe wall clearance box, first drive to the nearest safe point
        # fsm can then call this method again with the same target from the updated robot position.
        if not self.__points_equal(start_point, safe_start_point):
            return [start_point, safe_start_point]

        # pathfinder uses centimetres while callers use image pixels
        world_path = self.pathfinder.plan_smooth_path(
            self.__to_world_cm(start_point),
            self.__to_world_cm(drive_target_point),
            obstacles=self.__obstacles_to_world_cm(obstacles),
        )
        pixel_path = [self.__to_pixel(point) for point in world_path]

        if pixel_path:
            # preserve exact endpoints after the conversion
            pixel_path[0] = start_point
            pixel_path[-1] = drive_target_point
            return pixel_path

        return [start_point]
