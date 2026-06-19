from collections.abc import Mapping
from heapq import heappop, heappush
from itertools import count
from math import ceil, floor, hypot, sqrt

from utils.point import Point

from robot_logic.navigation_config import (
    GRID_SIZE_CM,
    ROBOT_RADIUS_CM,
    SEARCH_PADDING_CM,
    TURN_PENALTY_CM,
)
GridCell = tuple[int, int]
Direction = tuple[int, int]
PathState = tuple[GridCell, Direction | None]
SearchBounds = tuple[float, float, float, float]
class pathfinder:
    def __init__(self):
        # create a pathfinder instance
        pass



    @staticmethod
    def __distance(point_a: Point, point_b: Point) -> float:
        # return the /euclidean/ distance between two points
        return hypot(point_b.x - point_a.x, point_b.y - point_a.y)

    @staticmethod
    def __corner_to_point(value, obstacle_index: int, corner_index: int) -> Point:
        # convert one supported obstacle corner format into a Point
        if isinstance(value, Point):
            return value

        if hasattr(value, "x") and hasattr(value, "y"):
            return Point(value.x, value.y)

        try:
            return Point(value[0], value[1])
        except (TypeError, IndexError, KeyError, ValueError) as error:
            raise ValueError(
                "Invalid obstacle corner at "
                f"obstacles[{obstacle_index}][{corner_index}]: "
                "expected Point, (x, y), or object with .x and .y"
            ) from error


    @staticmethod
    def __is_corner_like(value) -> bool:
        # Check whether a value looks like one (x, y) corner.
        if isinstance(value, Point):
            return True

        if hasattr(value, "x") and hasattr(value, "y"):
            return True

        try:
            value[0]
            value[1]
            return len(value) == 2
        except (TypeError, IndexError, KeyError, ValueError):
            return False


    @staticmethod
    def __obstacle_items(obstacles) -> tuple:
        if isinstance(obstacles, Mapping):
            if "vertical_box" in obstacles or "horizontal_box" in obstacles:
                return tuple(
                    obstacles[key]
                    for key in ("vertical_box", "horizontal_box")
                    if key in obstacles and obstacles[key] is not None
                )

            return tuple(obstacles.values())

        try:
            obstacle_items = tuple(obstacles)
        except TypeError as error:
            raise ValueError(
                "Invalid obstacles: expected an iterable of 4-corner obstacles or None"
            ) from error

        if (
            len(obstacle_items) == 4
            and all(pathfinder.__is_corner_like(corner) for corner in obstacle_items)
        ):
            return (obstacle_items,)

        return obstacle_items


    def __normalize_obstacles(self, obstacles) -> tuple[tuple[Point, Point, Point, Point], ...]:
        # normalize optional obstacle input into 4-point polygons
        if obstacles is None:
            return ()

        obstacle_items = self.__obstacle_items(obstacles)

        normalized = []
        for obstacle_index, obstacle in enumerate(obstacle_items):
            try:
                corners = tuple(obstacle)
            except TypeError as error:
                raise ValueError(
                    f"Invalid obstacle at obstacles[{obstacle_index}]: "
                    "expected an iterable with exactly 4 corners"
                ) from error

            if len(corners) != 4:
                raise ValueError(
                    f"Invalid obstacle at obstacles[{obstacle_index}]: "
                    f"expected exactly 4 corners, got {len(corners)}"
                )

            normalized.append(
                tuple(
                    self.__corner_to_point(corner, obstacle_index, corner_index)
                    for corner_index, corner in enumerate(corners)
                )
            )

        return tuple(normalized)


    @staticmethod
    def __cross(point_a: Point, point_b: Point, point_c: Point) -> float:
        # return the signed area for three points
        return (
            (point_b.x - point_a.x) * (point_c.y - point_a.y)
            - (point_b.y - point_a.y) * (point_c.x - point_a.x)
        )


    @staticmethod
    def __point_on_segment(point: Point, segment_start: Point, segment_end: Point) -> bool:
        # check whether a point lies on a closed segment
        if pathfinder.__cross(segment_start, segment_end, point) != 0:
            return False

        return (
            min(segment_start.x, segment_end.x) <= point.x <= max(segment_start.x, segment_end.x)
            and min(segment_start.y, segment_end.y) <= point.y <= max(segment_start.y, segment_end.y)
        )


    def __point_to_segment_distance(
        self,
        point: Point,
        segment_start: Point,
        segment_end: Point,
    ) -> float:
        # return the shortest distance from a point to a segment
        dx = segment_end.x - segment_start.x
        dy = segment_end.y - segment_start.y
        segment_length_squared = dx * dx + dy * dy

        if segment_length_squared == 0:
            return self.__distance(point, segment_start)

        projection = (
            ((point.x - segment_start.x) * dx + (point.y - segment_start.y) * dy)
            / segment_length_squared
        )
        clamped_projection = min(1.0, max(0.0, projection))
        closest_x = segment_start.x + clamped_projection * dx
        closest_y = segment_start.y + clamped_projection * dy
        return hypot(point.x - closest_x, point.y - closest_y)


    def __point_in_polygon(self, point: Point, polygon: tuple[Point, Point, Point, Point]) -> bool:
        # ray-cast point-in-polygon check; polygon edges count as blocked
        inside = False
        previous = polygon[-1]

        for current in polygon:
            if self.__point_on_segment(point, previous, current):
                return True

            crosses_ray = (current.y > point.y) != (previous.y > point.y)
            if crosses_ray:
                intersection_x = (
                    (previous.x - current.x)
                    * (point.y - current.y)
                    / (previous.y - current.y)
                    + current.x
                )
                if point.x < intersection_x:
                    inside = not inside

            previous = current

        return inside


    def __point_to_polygon_distance(
        self,
        point: Point,
        polygon: tuple[Point, Point, Point, Point],
    ) -> float:
        # return the shortest distance from a point to a polygon
        if self.__point_in_polygon(point, polygon):
            return 0.0

        minimum_distance = float("inf")
        previous = polygon[-1]

        for current in polygon:
            minimum_distance = min(
                minimum_distance,
                self.__point_to_segment_distance(point, previous, current),
            )
            previous = current

        return minimum_distance


    @staticmethod
    def __polygon_bounds(polygon: tuple[Point, Point, Point, Point]) -> tuple[float, float, float, float]:
        return (
            min(point.x for point in polygon),
            max(point.x for point in polygon),
            min(point.y for point in polygon),
            max(point.y for point in polygon),
        )


    @staticmethod
    def __point_near_polygon_bounds(
        point: Point,
        polygon: tuple[Point, Point, Point, Point],
        padding: float,
    ) -> bool:
        min_x, max_x, min_y, max_y = pathfinder.__polygon_bounds(polygon)
        return (
            min_x - padding <= point.x <= max_x + padding
            and min_y - padding <= point.y <= max_y + padding
        )


    @staticmethod
    def __segment_near_polygon_bounds(
        start: Point,
        end: Point,
        polygon: tuple[Point, Point, Point, Point],
        padding: float,
    ) -> bool:
        min_x, max_x, min_y, max_y = pathfinder.__polygon_bounds(polygon)
        segment_min_x = min(start.x, end.x) - padding
        segment_max_x = max(start.x, end.x) + padding
        segment_min_y = min(start.y, end.y) - padding
        segment_max_y = max(start.y, end.y) + padding

        return not (
            segment_max_x < min_x
            or segment_min_x > max_x
            or segment_max_y < min_y
            or segment_min_y > max_y
        )


    def __segments_intersect(
        self,
        first_start: Point,
        first_end: Point,
        second_start: Point,
        second_end: Point,
    ) -> bool:
        # check whether two closed line segments touch or cross
        first_second_start = self.__cross(first_start, first_end, second_start)
        first_second_end = self.__cross(first_start, first_end, second_end)
        second_first_start = self.__cross(second_start, second_end, first_start)
        second_first_end = self.__cross(second_start, second_end, first_end)

        if first_second_start == 0 and self.__point_on_segment(second_start, first_start, first_end):
            return True
        if first_second_end == 0 and self.__point_on_segment(second_end, first_start, first_end):
            return True
        if second_first_start == 0 and self.__point_on_segment(first_start, second_start, second_end):
            return True
        if second_first_end == 0 and self.__point_on_segment(first_end, second_start, second_end):
            return True

        return (
            (first_second_start > 0) != (first_second_end > 0)
            and (second_first_start > 0) != (second_first_end > 0)
        )


    def __segment_to_segment_distance(
        self,
        first_start: Point,
        first_end: Point,
        second_start: Point,
        second_end: Point,
    ) -> float:
        # return the shortest distance between two segments
        if self.__segments_intersect(first_start, first_end, second_start, second_end):
            return 0.0

        return min(
            self.__point_to_segment_distance(first_start, second_start, second_end),
            self.__point_to_segment_distance(first_end, second_start, second_end),
            self.__point_to_segment_distance(second_start, first_start, first_end),
            self.__point_to_segment_distance(second_end, first_start, first_end),
        )


    def __segment_intersects_polygon(
        self,
        start: Point,
        end: Point,
        polygon: tuple[Point, Point, Point, Point],
    ) -> bool:
        # check whether a segment enters, leaves, touches, or stays inside a polygon
        if self.__point_in_polygon(start, polygon) or self.__point_in_polygon(end, polygon):
            return True

        previous = polygon[-1]
        for current in polygon:
            if self.__segments_intersect(start, end, previous, current):
                return True
            previous = current

        return False


    def __segment_to_polygon_distance(
        self,
        start: Point,
        end: Point,
        polygon: tuple[Point, Point, Point, Point],
    ) -> float:
        # return the shortest distance between a segment and a polygon
        if self.__segment_intersects_polygon(start, end, polygon):
            return 0.0

        minimum_distance = float("inf")
        previous = polygon[-1]

        for current in polygon:
            minimum_distance = min(
                minimum_distance,
                self.__segment_to_segment_distance(start, end, previous, current),
            )
            previous = current

        return minimum_distance


    def __is_blocked_by_obstacles(self, point: Point, obstacles: tuple) -> bool:
        # check whether any obstacle blocks a point
        for obstacle in obstacles:
            if not self.__point_near_polygon_bounds(point, obstacle, ROBOT_RADIUS_CM):
                continue

            if (
                self.__point_in_polygon(point, obstacle)
                or self.__point_to_polygon_distance(point, obstacle) <= ROBOT_RADIUS_CM
            ):
                return True

        return False


    def __is_segment_blocked_by_obstacles(self, start: Point, end: Point, obstacles: tuple) -> bool:
        # check whether a segment intersects any obstacle polygon
        for obstacle in obstacles:
            if not self.__segment_near_polygon_bounds(start, end, obstacle, ROBOT_RADIUS_CM):
                continue

            if (
                self.__segment_intersects_polygon(start, end, obstacle)
                or self.__segment_to_polygon_distance(start, end, obstacle) <= ROBOT_RADIUS_CM
            ):
                return True

        return False

    @staticmethod
    def __make_search_bounds(
        start: Point,
        target: Point,
        padding_cm: float = SEARCH_PADDING_CM,
        grid_size_cm: float = GRID_SIZE_CM,
    ) -> SearchBounds:
        # build padded grid-aligned bounds around start and target
        min_x = floor((min(start.x, target.x) - padding_cm) / grid_size_cm) * grid_size_cm
        max_x = ceil((max(start.x, target.x) + padding_cm) / grid_size_cm) * grid_size_cm
        min_y = floor((min(start.y, target.y) - padding_cm) / grid_size_cm) * grid_size_cm
        max_y = ceil((max(start.y, target.y) + padding_cm) / grid_size_cm) * grid_size_cm
        return min_x, max_x, min_y, max_y

    @staticmethod
    def __grid_limits(bounds: SearchBounds, grid_size_cm: float = GRID_SIZE_CM) -> GridCell:
        # return the maximum grid indices inside the bounds
        min_x, max_x, min_y, max_y = bounds
        return (
            int(round((max_x - min_x) / grid_size_cm)),
            int(round((max_y - min_y) / grid_size_cm)),
        )

    @staticmethod
    def __point_to_grid(
        point: Point,
        bounds: SearchBounds,
        grid_size_cm: float = GRID_SIZE_CM,
    ) -> GridCell:
        # convert a world point to a grid cell
        min_x, _, min_y, _ = bounds
        return (
            int(round((point.x - min_x) / grid_size_cm)),
            int(round((point.y - min_y) / grid_size_cm)),
        )

    @staticmethod
    def __grid_to_point(
        cell: GridCell,
        bounds: SearchBounds,
        grid_size_cm: float = GRID_SIZE_CM,
    ) -> Point:
        # convert a grid cell to a world point
        min_x, _, min_y, _ = bounds
        return Point(
            min_x + cell[0] * grid_size_cm,
            min_y + cell[1] * grid_size_cm,
        )


    def __is_cell_blocked(self ,
        cell: GridCell,
        bounds: SearchBounds,
        obstacles: tuple,
        grid_size_cm: float = GRID_SIZE_CM,
        max_cell: GridCell | None = None,
        blocked_cell_cache: dict[GridCell, bool] | None = None,
    ) -> bool:
        # check whether a grid cell is outside bounds or blocked.
        if max_cell is None:
            max_cell = self.__grid_limits(bounds, grid_size_cm)

        max_x, max_y = max_cell

        if cell[0] < 0 or cell[0] > max_x or cell[1] < 0 or cell[1] > max_y:
            return True

        if blocked_cell_cache is None:
            return self.__is_blocked_by_obstacles(
                self.__grid_to_point(cell, bounds, grid_size_cm),
                obstacles,
            )

        if cell not in blocked_cell_cache:
            blocked_cell_cache[cell] = self.__is_blocked_by_obstacles(
                self.__grid_to_point(cell, bounds, grid_size_cm),
                obstacles,
            )

        return blocked_cell_cache[cell]


    def __is_point_blocked(self, point: Point, obstacles=None) -> bool:
        # check whether a world point is blocked.
        return self.__is_blocked_by_obstacles(point, self.__normalize_obstacles(obstacles))


    def __cell_neighbors(
        self,
        cell: GridCell,
        bounds: SearchBounds,
        obstacles: tuple,
        grid_size_cm: float,
        max_cell: GridCell,
        blocked_cell_cache: dict[GridCell, bool],
    ) -> list[tuple[GridCell, float, Direction]]:
        # return reachable neighboring cells with movement costs.
        neighbors = []
        directions = [
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        ]
        current_point = self.__grid_to_point(cell, bounds, grid_size_cm)

        for dx, dy in directions:
            next_cell = (cell[0] + dx, cell[1] + dy)

            if self.__is_cell_blocked(
                next_cell,
                bounds,
                obstacles,
                grid_size_cm,
                max_cell,
                blocked_cell_cache,
            ):
                continue

            next_point = self.__grid_to_point(next_cell, bounds, grid_size_cm)
            if self.__is_segment_blocked_by_obstacles(current_point, next_point, obstacles):
                continue

            move_cost = grid_size_cm * (sqrt(2) if dx != 0 and dy != 0 else 1.0)
            neighbors.append((next_cell, move_cost, (dx, dy)))

        return neighbors

    @staticmethod
    def __heuristic(cell: GridCell, target: GridCell, grid_size_cm: float = GRID_SIZE_CM) -> float:
        # estimate remaining path cost from a cell to the target
        return hypot(target[0] - cell[0], target[1] - cell[1]) * grid_size_cm


    def __rebuild_path(
        self,
        came_from: dict[PathState, PathState],
        start: PathState,
        target: PathState,
        bounds: SearchBounds,
        grid_size_cm: float,
    ) -> list[Point]:
        # rebuild a point path from the A* parent map
        current = target
        cells = [current[0]]

        while current != start:
            current = came_from[current]
            cells.append(current[0])

        cells.reverse()
        return [self.__grid_to_point(cell, bounds, grid_size_cm) for cell in cells]


    def __plan_path(
        self,
        start: Point,
        target: Point,
        obstacles=None,
        grid_size_cm: float = GRID_SIZE_CM,
        turn_penalty_cm: float = TURN_PENALTY_CM,
        search_padding_cm: float = SEARCH_PADDING_CM,
        normalized_obstacles: tuple | None = None,
    ) -> list[Point]:
        # find a raw grid path from start to target with A*
        if normalized_obstacles is None:
            normalized_obstacles = self.__normalize_obstacles(obstacles)

        bounds = self.__make_search_bounds(start, target, search_padding_cm, grid_size_cm)
        max_cell = self.__grid_limits(bounds, grid_size_cm)
        blocked_cell_cache: dict[GridCell, bool] = {}
        start_cell = self.__point_to_grid(start, bounds, grid_size_cm)
        target_cell = self.__point_to_grid(target, bounds, grid_size_cm)

        if self.__is_cell_blocked(
            start_cell,
            bounds,
            normalized_obstacles,
            grid_size_cm,
            max_cell,
            blocked_cell_cache,
        ):
            return []

        if self.__is_cell_blocked(
            target_cell,
            bounds,
            normalized_obstacles,
            grid_size_cm,
            max_cell,
            blocked_cell_cache,
        ):
            return []

        start_state: PathState = (start_cell, None)
        open_cells = []
        tie_breaker = count()
        heappush(open_cells, (0.0, next(tie_breaker), 0.0, start_state))

        came_from: dict[PathState, PathState] = {}
        best_cost = {start_state: 0.0}

        while open_cells:
            _, _, current_cost, current_state = heappop(open_cells)
            current, previous_direction = current_state

            if current_cost > best_cost.get(current_state, float("inf")):
                continue

            if current == target_cell:
                path = self.__rebuild_path(came_from, start_state, current_state, bounds, grid_size_cm)
                path[0] = start
                path[-1] = target
                return path

            for neighbor, move_cost, move_direction in self.__cell_neighbors(
                current,
                bounds,
                normalized_obstacles,
                grid_size_cm,
                max_cell,
                blocked_cell_cache,
            ):
                turn_cost = 0.0
                if previous_direction is not None and previous_direction != move_direction:
                    turn_cost = turn_penalty_cm

                next_state: PathState = (neighbor, move_direction)
                new_cost = best_cost[current_state] + move_cost + turn_cost

                if next_state not in best_cost or new_cost < best_cost[next_state]:
                    best_cost[next_state] = new_cost
                    priority = new_cost + self.__heuristic(neighbor, target_cell, grid_size_cm)
                    came_from[next_state] = current_state
                    heappush(open_cells, (priority, next(tie_breaker), new_cost, next_state))

        return []


    def __path_has_line_of_sight(
        self,
        start: Point,
        end: Point,
        normalized_obstacles: tuple,
    ) -> bool:
        # check whether a straight segment avoids all obstacles
        return not self.__is_segment_blocked_by_obstacles(start, end, normalized_obstacles)


    def __smooth_path(
        self,
        raw_path: list[Point],
        obstacles=None,
        normalized_obstacles: tuple | None = None,
    ) -> list[Point]:
        # simplify a path by skipping points with clear line of sight
        if len(raw_path) <= 2:
            return raw_path[:]

        if normalized_obstacles is None:
            normalized_obstacles = self.__normalize_obstacles(obstacles)

        simplified = [raw_path[0]]
        current_index = 0

        while current_index < len(raw_path) - 1:
            next_index = current_index + 1

            for candidate_index in range(len(raw_path) - 1, current_index, -1):
                if self.__path_has_line_of_sight(
                    raw_path[current_index],
                    raw_path[candidate_index],
                    normalized_obstacles,
                ):
                    next_index = candidate_index
                    break

            simplified.append(raw_path[next_index])
            current_index = next_index

        return simplified


    def plan_smooth_path(
        self,
        start: Point,
        target: Point,
        obstacles=None,
        grid_size_cm: float = GRID_SIZE_CM,
        turn_penalty_cm: float = TURN_PENALTY_CM,
        search_padding_cm: float = SEARCH_PADDING_CM,
    ) -> list[Point]:
        # plan and smooth a path from start to target
        normalized_obstacles = self.__normalize_obstacles(obstacles)
        if not self.__is_segment_blocked_by_obstacles(start, target, normalized_obstacles):
            return [start, target]

        raw_path = self.__plan_path(
            start,
            target,
            obstacles,
            grid_size_cm,
            turn_penalty_cm,
            search_padding_cm,
            normalized_obstacles,
        )
        return self.__smooth_path(raw_path, obstacles, normalized_obstacles)


    def path_length(
        self,
        path: list[Point]
    ) -> float:
        # return the total length of a path
        total = 0.0

        for start, end in zip(path, path[1:]):
            total += self.__distance(start, end)

        return total
