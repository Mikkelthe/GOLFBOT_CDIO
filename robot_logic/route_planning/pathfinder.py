from heapq import heappop, heappush
from itertools import count
from math import hypot, sqrt
from robot_logic.navigation_config import (
    BALL_AVOID_RADIUS_CM,
    CROSS_FALLBACK_RADIUS_CM,
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
    PATH_SMOOTH_SAMPLE_STEP_CM,
    TURN_PENALTY_CM,
    WALL_SAFETY_MARGIN_CM,
)
from robot_logic.route_planning.obstacles import CrossObstacle
from robot_logic.route_planning.route_planner import (
    Point,
    distance,
)


GRID_SIZE_CM = 5.0

GridCell = tuple[int, int]
Direction = tuple[int, int]
PathState = tuple[GridCell, Direction | None]


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
    cross_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
    cross_obstacle: CrossObstacle | None = None,
    blocked_points: list[Point] | None = None,
    blocked_point_radius_cm: float = BALL_AVOID_RADIUS_CM,
) -> bool:
    max_x, max_y = grid_limits(grid_size_cm, wall_margin_cm)

    if cell[0] < 0 or cell[0] > max_x or cell[1] < 0 or cell[1] > max_y:
        return True

    point = grid_to_point(cell, grid_size_cm, wall_margin_cm)
    if cross_obstacle is not None and cross_obstacle.contains(point):
        return True

    if blocked_points:
        for blocked_point in blocked_points:
            if distance(point, blocked_point) <= blocked_point_radius_cm:
                return True

    if cross_center is None:
        return False

    return distance(point, cross_center) <= cross_radius_cm


def is_point_blocked(
    point: Point,
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
    cross_obstacle: CrossObstacle | None = None,
    blocked_points: list[Point] | None = None,
    blocked_point_radius_cm: float = BALL_AVOID_RADIUS_CM,
) -> bool:
    if (
        point.x < wall_margin_cm
        or point.x > FIELD_WIDTH_CM - wall_margin_cm
        or point.y < wall_margin_cm
        or point.y > FIELD_HEIGHT_CM - wall_margin_cm
    ):
        return True

    if cross_obstacle is not None and cross_obstacle.contains(point):
        return True

    if blocked_points:
        for blocked_point in blocked_points:
            if distance(point, blocked_point) <= blocked_point_radius_cm:
                return True

    if cross_center is None:
        return False

    return distance(point, cross_center) <= cross_radius_cm


def cell_neighbors(
    cell: GridCell,
    cross_center: Point | None,
    cross_radius_cm: float,
    grid_size_cm: float,
    wall_margin_cm: float,
    cross_obstacle: CrossObstacle | None,
    blocked_points: list[Point] | None,
    blocked_point_radius_cm: float,
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

        if is_cell_blocked(
            next_cell,
            cross_center,
            cross_radius_cm,
            grid_size_cm,
            wall_margin_cm,
            cross_obstacle,
            blocked_points,
            blocked_point_radius_cm,
        ):
            continue

        move_cost = grid_size_cm * (sqrt(2) if dx != 0 and dy != 0 else 1.0)
        neighbors.append((next_cell, move_cost, (dx, dy)))

    return neighbors


def heuristic(cell: GridCell, goal: GridCell, grid_size_cm: float = GRID_SIZE_CM) -> float:
    return hypot(goal[0] - cell[0], goal[1] - cell[1]) * grid_size_cm


def rebuild_path(
    came_from: dict[PathState, PathState],
    start: PathState,
    goal: PathState,
    grid_size_cm: float,
    wall_margin_cm: float,
) -> list[Point]:
    current = goal
    cells = [current[0]]

    while current != start:
        current = came_from[current]
        cells.append(current[0])

    cells.reverse()
    return [grid_to_point(cell, grid_size_cm, wall_margin_cm) for cell in cells]


def plan_path(
    start: Point,
    goal: Point,
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
    cross_obstacle: CrossObstacle | None = None,
    blocked_points: list[Point] | None = None,
    blocked_point_radius_cm: float = BALL_AVOID_RADIUS_CM,
    turn_penalty_cm: float = TURN_PENALTY_CM,
) -> list[Point]:
    safe_start = clamp_to_field(start, wall_margin_cm)
    safe_goal = clamp_to_field(goal, wall_margin_cm)
    start_cell = point_to_grid(safe_start, grid_size_cm, wall_margin_cm)
    goal_cell = point_to_grid(safe_goal, grid_size_cm, wall_margin_cm)

    if is_cell_blocked(
        start_cell,
        cross_center,
        cross_radius_cm,
        grid_size_cm,
        wall_margin_cm,
        cross_obstacle,
        blocked_points,
        blocked_point_radius_cm,
    ):
        return []

    if is_cell_blocked(
        goal_cell,
        cross_center,
        cross_radius_cm,
        grid_size_cm,
        wall_margin_cm,
        cross_obstacle,
        blocked_points,
        blocked_point_radius_cm,
    ):
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

        if current == goal_cell:
            path = rebuild_path(came_from, start_state, current_state, grid_size_cm, wall_margin_cm)
            path[0] = safe_start
            path[-1] = safe_goal
            return path

        for neighbor, move_cost, move_direction in cell_neighbors(
            current,
            cross_center,
            cross_radius_cm,
            grid_size_cm,
            wall_margin_cm,
            cross_obstacle,
            blocked_points,
            blocked_point_radius_cm,
        ):
            turn_cost = 0.0
            if previous_direction is not None and previous_direction != move_direction:
                turn_cost = turn_penalty_cm

            next_state: PathState = (neighbor, move_direction)
            new_cost = best_cost[current_state] + move_cost + turn_cost

            if next_state not in best_cost or new_cost < best_cost[next_state]:
                best_cost[next_state] = new_cost
                priority = new_cost + heuristic(neighbor, goal_cell, grid_size_cm)
                came_from[next_state] = current_state
                heappush(open_cells, (priority, next(tie_breaker), next_state))

    return []


def path_has_line_of_sight(
    start: Point,
    end: Point,
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
    cross_obstacle: CrossObstacle | None = None,
    blocked_points: list[Point] | None = None,
    blocked_point_radius_cm: float = BALL_AVOID_RADIUS_CM,
    sample_step_cm: float = PATH_SMOOTH_SAMPLE_STEP_CM,
) -> bool:
    segment_length = distance(start, end)
    if segment_length == 0:
        return not is_point_blocked(
            start,
            cross_center,
            cross_radius_cm,
            wall_margin_cm,
            cross_obstacle,
            blocked_points,
            blocked_point_radius_cm,
        )

    samples = max(1, int(segment_length / sample_step_cm))
    for index in range(samples + 1):
        t = index / samples
        point = Point(
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
        )
        if is_point_blocked(
            point,
            cross_center,
            cross_radius_cm,
            wall_margin_cm,
            cross_obstacle,
            blocked_points,
            blocked_point_radius_cm,
        ):
            return False

    return True


def smooth_path(
    raw_path: list[Point],
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
    cross_obstacle: CrossObstacle | None = None,
    blocked_points: list[Point] | None = None,
    blocked_point_radius_cm: float = BALL_AVOID_RADIUS_CM,
) -> list[Point]:
    if len(raw_path) <= 2:
        return raw_path[:]

    simplified = [raw_path[0]]
    current_index = 0

    while current_index < len(raw_path) - 1:
        next_index = current_index + 1

        for candidate_index in range(len(raw_path) - 1, current_index, -1):
            if path_has_line_of_sight(
                raw_path[current_index],
                raw_path[candidate_index],
                cross_center,
                cross_radius_cm,
                wall_margin_cm,
                cross_obstacle,
                blocked_points,
                blocked_point_radius_cm,
            ):
                next_index = candidate_index
                break

        simplified.append(raw_path[next_index])
        current_index = next_index

    return simplified


def plan_smooth_path(
    start: Point,
    goal: Point,
    cross_center: Point | None = None,
    cross_radius_cm: float = CROSS_FALLBACK_RADIUS_CM,
    grid_size_cm: float = GRID_SIZE_CM,
    wall_margin_cm: float = WALL_SAFETY_MARGIN_CM,
    cross_obstacle: CrossObstacle | None = None,
    blocked_points: list[Point] | None = None,
    blocked_point_radius_cm: float = BALL_AVOID_RADIUS_CM,
    turn_penalty_cm: float = TURN_PENALTY_CM,
) -> list[Point]:
    raw_path = plan_path(
        start,
        goal,
        cross_center,
        cross_radius_cm,
        grid_size_cm,
        wall_margin_cm,
        cross_obstacle,
        blocked_points,
        blocked_point_radius_cm,
        turn_penalty_cm,
    )
    return smooth_path(
        raw_path,
        cross_center,
        cross_radius_cm,
        wall_margin_cm,
        cross_obstacle,
        blocked_points,
        blocked_point_radius_cm,
    )


def path_length(path: list[Point]) -> float:
    total = 0.0

    for start, end in zip(path, path[1:]):
        total += distance(start, end)

    return total

