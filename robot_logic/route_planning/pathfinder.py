from heapq import heappop, heappush
from itertools import count
from math import hypot, sqrt

from robot_logic.route_planning.route_planner import (
    CROSS_SAFETY_RADIUS_CM,
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
    Point,
    distance,
)


GRID_SIZE_CM = 5.0
WALL_SAFETY_MARGIN_CM = 5.0

GridCell = tuple[int, int]


def clamp_value(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def clamp_to_field(point: Point, wall_margin_cm: float = WALL_SAFETY_MARGIN_CM) -> Point:
    # ArUco can put the robot marker slightly outside the field; pathfinding starts at the nearest safe point.
    return Point(
        clamp_value(point.x, wall_margin_cm, FIELD_WIDTH_CM - wall_margin_cm),
        clamp_value(point.y, wall_margin_cm, FIELD_HEIGHT_CM - wall_margin_cm),
    )


def grid_limits(grid_size_cm: float = GRID_SIZE_CM, wall_margin_cm: float = WALL_SAFETY_MARGIN_CM) -> GridCell:
    max_x = int(round((FIELD_WIDTH_CM - 2 * wall_margin_cm) / grid_size_cm))
    max_y = int(round((FIELD_HEIGHT_CM - 2 * wall_margin_cm) / grid_size_cm))
    return max_x, max_y


def point_to_grid(
    point: Point,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
) -> GridCell:
    point = clamp_to_field(point, wall_margin_cm)
    return (
        int(round((point.x - wall_margin_cm) / grid_size_cm)),
        int(round((point.y - wall_margin_cm) / grid_size_cm)),
    )


def grid_to_point(
    cell: GridCell,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
) -> Point:
    return Point(
        wall_margin_cm + cell[0] * grid_size_cm,
        wall_margin_cm + cell[1] * grid_size_cm,
    )


def is_cell_blocked(
    cell: GridCell,
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_SAFETY_RADIUS_CM,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
) -> bool:
    max_x, max_y = grid_limits(grid_size_cm, wall_margin_cm)

    if cell[0] < 0 or cell[0] > max_x or cell[1] < 0 or cell[1] > max_y:
        return True

    if cross_center is None:
        return False

    return distance(grid_to_point(cell, grid_size_cm, wall_margin_cm), cross_center) <= cross_radius_cm


def cell_neighbors(
    cell: GridCell,
    cross_center: Point | None,
    cross_radius_cm: float,
    grid_size_cm: float,
    wall_margin_cm: float,
) -> list[tuple[GridCell, float]]:
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

        if is_cell_blocked(next_cell, cross_center, cross_radius_cm, grid_size_cm, wall_margin_cm):
            continue

        move_cost = grid_size_cm * (sqrt(2) if dx != 0 and dy != 0 else 1.0)
        neighbors.append((next_cell, move_cost))

    return neighbors


def heuristic(cell: GridCell, goal: GridCell, grid_size_cm: float = GRID_SIZE_CM) -> float:
    return hypot(goal[0] - cell[0], goal[1] - cell[1]) * grid_size_cm


def rebuild_path(
    came_from: dict[GridCell, GridCell],
    start: GridCell,
    goal: GridCell,
    grid_size_cm: float,
    wall_margin_cm: float,
) -> list[Point]:
    current = goal
    cells = [current]

    while current != start:
        current = came_from[current]
        cells.append(current)

    cells.reverse()
    return [grid_to_point(cell, grid_size_cm, wall_margin_cm) for cell in cells]


def plan_path(
    start: Point,
    goal: Point,
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_SAFETY_RADIUS_CM,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
) -> list[Point]:
    safe_start = clamp_to_field(start, wall_margin_cm)
    safe_goal = clamp_to_field(goal, wall_margin_cm)
    start_cell = point_to_grid(safe_start, grid_size_cm, wall_margin_cm)
    goal_cell = point_to_grid(safe_goal, grid_size_cm, wall_margin_cm)

    if is_cell_blocked(start_cell, cross_center, cross_radius_cm, grid_size_cm, wall_margin_cm):
        return []

    if is_cell_blocked(goal_cell, cross_center, cross_radius_cm, grid_size_cm, wall_margin_cm):
        return []

    open_cells = []
    tie_breaker = count()
    heappush(open_cells, (0.0, next(tie_breaker), start_cell))

    came_from: dict[GridCell, GridCell] = {}
    best_cost = {start_cell: 0.0}

    while open_cells:
        _, _, current = heappop(open_cells)

        if current == goal_cell:
            path = rebuild_path(came_from, start_cell, goal_cell, grid_size_cm, wall_margin_cm)
            path[0] = safe_start
            path[-1] = safe_goal
            return path

        for neighbor, move_cost in cell_neighbors(
            current,
            cross_center,
            cross_radius_cm,
            grid_size_cm,
            wall_margin_cm,
        ):
            new_cost = best_cost[current] + move_cost

            if neighbor not in best_cost or new_cost < best_cost[neighbor]:
                best_cost[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal_cell, grid_size_cm)
                came_from[neighbor] = current
                heappush(open_cells, (priority, next(tie_breaker), neighbor))

    return []


def path_length(path: list[Point]) -> float:
    total = 0.0

    for start, end in zip(path, path[1:]):
        total += distance(start, end)

    return total


def straight_line_crosses_circle(start: Point, end: Point, center: Point, radius_cm: float) -> bool:
    dx = end.x - start.x
    dy = end.y - start.y
    segment_length_squared = dx * dx + dy * dy

    if segment_length_squared == 0:
        return distance(start, center) <= radius_cm

    t = ((center.x - start.x) * dx + (center.y - start.y) * dy) / segment_length_squared
    t = clamp_value(t, 0.0, 1.0)
    closest = Point(start.x + t * dx, start.y + t * dy)
    return distance(closest, center) <= radius_cm
