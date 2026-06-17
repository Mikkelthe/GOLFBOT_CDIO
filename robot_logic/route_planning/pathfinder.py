from heapq import heappop, heappush
from itertools import count
from math import ceil, floor, hypot, sqrt

from utils.point import Point

from robot_logic.navigation_config import (
    GRID_SIZE_CM,
    PATH_SMOOTH_SAMPLE_STEP_CM,
    SEARCH_PADDING_CM,
    TURN_PENALTY_CM,
)


GridCell = tuple[int, int]
Direction = tuple[int, int]
PathState = tuple[GridCell, Direction | None]
SearchBounds = tuple[float, float, float, float]


def distance(point_a: Point, point_b: Point) -> float:
    return hypot(point_b.x - point_a.x, point_b.y - point_a.y)


def _has_obstacle_interface(value) -> bool:
    return callable(getattr(value, "contains", None)) or callable(getattr(value, "blocks", None))


def _iter_obstacles(obstacles) -> tuple:
    if obstacles is None:
        return ()
    if _has_obstacle_interface(obstacles):
        return (obstacles,)
    return tuple(obstacles)


def _obstacle_blocks_point(obstacle, point: Point) -> bool:
    contains = getattr(obstacle, "contains", None)
    if callable(contains):
        return bool(contains(point))

    blocks = getattr(obstacle, "blocks", None)
    if callable(blocks):
        return bool(blocks(point))

    return False


def _is_blocked_by_obstacles(point: Point, obstacles: tuple) -> bool:
    return any(_obstacle_blocks_point(obstacle, point) for obstacle in obstacles)


def make_search_bounds(
    start: Point,
    target: Point,
    padding_cm: float = SEARCH_PADDING_CM,
    grid_size_cm: float = GRID_SIZE_CM,
) -> SearchBounds:
    min_x = floor((min(start.x, target.x) - padding_cm) / grid_size_cm) * grid_size_cm
    max_x = ceil((max(start.x, target.x) + padding_cm) / grid_size_cm) * grid_size_cm
    min_y = floor((min(start.y, target.y) - padding_cm) / grid_size_cm) * grid_size_cm
    max_y = ceil((max(start.y, target.y) + padding_cm) / grid_size_cm) * grid_size_cm
    return min_x, max_x, min_y, max_y


def grid_limits(bounds: SearchBounds, grid_size_cm: float = GRID_SIZE_CM) -> GridCell:
    min_x, max_x, min_y, max_y = bounds
    return (
        int(round((max_x - min_x) / grid_size_cm)),
        int(round((max_y - min_y) / grid_size_cm)),
    )


def point_to_grid(
    point: Point,
    bounds: SearchBounds,
    grid_size_cm: float = GRID_SIZE_CM,
) -> GridCell:
    min_x, _, min_y, _ = bounds
    return (
        int(round((point.x - min_x) / grid_size_cm)),
        int(round((point.y - min_y) / grid_size_cm)),
    )


def grid_to_point(
    cell: GridCell,
    bounds: SearchBounds,
    grid_size_cm: float = GRID_SIZE_CM,
) -> Point:
    min_x, _, min_y, _ = bounds
    return Point(
        min_x + cell[0] * grid_size_cm,
        min_y + cell[1] * grid_size_cm,
    )


def is_cell_blocked(
    cell: GridCell,
    bounds: SearchBounds,
    obstacles: tuple,
    grid_size_cm: float = GRID_SIZE_CM,
) -> bool:
    max_x, max_y = grid_limits(bounds, grid_size_cm)

    if cell[0] < 0 or cell[0] > max_x or cell[1] < 0 or cell[1] > max_y:
        return True

    return _is_blocked_by_obstacles(grid_to_point(cell, bounds, grid_size_cm), obstacles)


def is_point_blocked(point: Point, obstacles=None) -> bool:
    return _is_blocked_by_obstacles(point, _iter_obstacles(obstacles))


def cell_neighbors(
    cell: GridCell,
    bounds: SearchBounds,
    obstacles: tuple,
    grid_size_cm: float,
) -> list[tuple[GridCell, float, Direction]]:
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

    for dx, dy in directions:
        next_cell = (cell[0] + dx, cell[1] + dy)

        if is_cell_blocked(next_cell, bounds, obstacles, grid_size_cm):
            continue

        move_cost = grid_size_cm * (sqrt(2) if dx != 0 and dy != 0 else 1.0)
        neighbors.append((next_cell, move_cost, (dx, dy)))

    return neighbors


def heuristic(cell: GridCell, target: GridCell, grid_size_cm: float = GRID_SIZE_CM) -> float:
    return hypot(target[0] - cell[0], target[1] - cell[1]) * grid_size_cm


def rebuild_path(
    came_from: dict[PathState, PathState],
    start: PathState,
    target: PathState,
    bounds: SearchBounds,
    grid_size_cm: float,
) -> list[Point]:
    current = target
    cells = [current[0]]

    while current != start:
        current = came_from[current]
        cells.append(current[0])

    cells.reverse()
    return [grid_to_point(cell, bounds, grid_size_cm) for cell in cells]


def plan_path(
    start: Point,
    target: Point,
    obstacles=None,
    grid_size_cm: float = GRID_SIZE_CM,
    turn_penalty_cm: float = TURN_PENALTY_CM,
    search_padding_cm: float = SEARCH_PADDING_CM,
) -> list[Point]:
    normalized_obstacles = _iter_obstacles(obstacles)
    bounds = make_search_bounds(start, target, search_padding_cm, grid_size_cm)
    start_cell = point_to_grid(start, bounds, grid_size_cm)
    target_cell = point_to_grid(target, bounds, grid_size_cm)

    if is_cell_blocked(start_cell, bounds, normalized_obstacles, grid_size_cm):
        return []

    if is_cell_blocked(target_cell, bounds, normalized_obstacles, grid_size_cm):
        return []

    start_state: PathState = (start_cell, None)
    open_cells = []
    tie_breaker = count()
    heappush(open_cells, (0.0, next(tie_breaker), start_state))

    came_from: dict[PathState, PathState] = {}
    best_cost = {start_state: 0.0}

    while open_cells:
        _, _, current_state = heappop(open_cells)
        current, previous_direction = current_state

        if current == target_cell:
            path = rebuild_path(came_from, start_state, current_state, bounds, grid_size_cm)
            path[0] = start
            path[-1] = target
            return path

        for neighbor, move_cost, move_direction in cell_neighbors(
            current,
            bounds,
            normalized_obstacles,
            grid_size_cm,
        ):
            turn_cost = 0.0
            if previous_direction is not None and previous_direction != move_direction:
                turn_cost = turn_penalty_cm

            next_state: PathState = (neighbor, move_direction)
            new_cost = best_cost[current_state] + move_cost + turn_cost

            if next_state not in best_cost or new_cost < best_cost[next_state]:
                best_cost[next_state] = new_cost
                priority = new_cost + heuristic(neighbor, target_cell, grid_size_cm)
                came_from[next_state] = current_state
                heappush(open_cells, (priority, next(tie_breaker), next_state))

    return []


def path_has_line_of_sight(
    start: Point,
    end: Point,
    obstacles=None,
    sample_step_cm: float = PATH_SMOOTH_SAMPLE_STEP_CM,
) -> bool:
    normalized_obstacles = _iter_obstacles(obstacles)
    segment_length = distance(start, end)
    if segment_length == 0:
        return not _is_blocked_by_obstacles(start, normalized_obstacles)

    samples = max(1, int(segment_length / sample_step_cm))
    for index in range(samples + 1):
        t = index / samples
        point = Point(
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
        )
        if _is_blocked_by_obstacles(point, normalized_obstacles):
            return False

    return True


def smooth_path(
    raw_path: list[Point],
    obstacles=None,
) -> list[Point]:
    if len(raw_path) <= 2:
        return raw_path[:]

    simplified = [raw_path[0]]
    current_index = 0

    while current_index < len(raw_path) - 1:
        next_index = current_index + 1

        for candidate_index in range(len(raw_path) - 1, current_index, -1):
            if path_has_line_of_sight(raw_path[current_index], raw_path[candidate_index], obstacles):
                next_index = candidate_index
                break

        simplified.append(raw_path[next_index])
        current_index = next_index

    return simplified


def plan_smooth_path(
    start: Point,
    target: Point,
    obstacles=None,
    grid_size_cm: float = GRID_SIZE_CM,
    turn_penalty_cm: float = TURN_PENALTY_CM,
    search_padding_cm: float = SEARCH_PADDING_CM,
) -> list[Point]:
    raw_path = plan_path(
        start,
        target,
        obstacles,
        grid_size_cm,
        turn_penalty_cm,
        search_padding_cm,
    )
    return smooth_path(raw_path, obstacles)


def path_length(path: list[Point]) -> float:
    total = 0.0

    for start, end in zip(path, path[1:]):
        total += distance(start, end)

    return total
