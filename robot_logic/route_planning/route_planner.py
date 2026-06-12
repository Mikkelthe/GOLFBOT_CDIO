from dataclasses import dataclass
from itertools import permutations
from math import atan2, degrees, hypot, isinf

from robot_logic.navigation_config import (
    BALL_AVOID_RADIUS_CM,
    EDGE_MARGIN_CM,
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
    GOAL_A_X_CM,
    GOAL_A_Y_CM,
    MAX_EXACT_ROUTE_TARGETS,
    ON_PATH_BALL_RADIUS_CM,
    PICKUP_APPROACH_DISTANCE_CM,
    ROUTE_STRATEGY,
    QUADRANT_SWEEP_ORDER,
    SAFE_PICKUP_MARGIN_CM,
    TURN_PENALTY_CM,
    WALL_CLUSTER_DISTANCE_CM,
    WALL_CLUSTER_ORDER_PENALTY_CM,
    WALL_PICKUP_MARGIN_CM,
    WALL_SIDE_TIE_MARGIN_CM,
)


Quadrant = str
WallSide = str
ALL_QUADRANTS: tuple[Quadrant, ...] = ("top_left", "top_right", "bottom_left", "bottom_right")
WALL_SIDE_PRIORITY: tuple[WallSide, ...] = ("right", "left", "bottom", "top")


@dataclass(frozen=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class Ball:
    name: str
    position: Point
    is_vip: bool = False


@dataclass(frozen=True)
class Goal:
    name: str
    position: Point


@dataclass(frozen=True)
class PickupTarget:
    ball: Ball
    pickup_point: Point
    face_direction: str
    is_wall_pickup: bool = False


def distance(point_a: Point, point_b: Point) -> float:
    return hypot(point_b.x - point_a.x, point_b.y - point_a.y)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def clamp_point(point: Point, margin_cm: float = SAFE_PICKUP_MARGIN_CM) -> Point:
    return Point(
        clamp(point.x, margin_cm, FIELD_WIDTH_CM - margin_cm),
        clamp(point.y, margin_cm, FIELD_HEIGHT_CM - margin_cm),
    )


def point_inside_field(point: Point, margin_cm: float = SAFE_PICKUP_MARGIN_CM) -> bool:
    return (
        margin_cm <= point.x <= FIELD_WIDTH_CM - margin_cm
        and margin_cm <= point.y <= FIELD_HEIGHT_CM - margin_cm
    )


def get_quadrant(point: Point) -> Quadrant:
    horizontal = "left" if point.x < FIELD_WIDTH_CM / 2.0 else "right"
    vertical = "top" if point.y >= FIELD_HEIGHT_CM / 2.0 else "bottom"
    return f"{vertical}_{horizontal}"


def _wall_distances(point: Point) -> dict[WallSide, float]:
    distances: dict[WallSide, float] = {}

    if point.x <= EDGE_MARGIN_CM:
        distances["left"] = point.x
    if point.x >= FIELD_WIDTH_CM - EDGE_MARGIN_CM:
        distances["right"] = FIELD_WIDTH_CM - point.x
    if point.y <= EDGE_MARGIN_CM:
        distances["bottom"] = point.y
    if point.y >= FIELD_HEIGHT_CM - EDGE_MARGIN_CM:
        distances["top"] = FIELD_HEIGHT_CM - point.y

    return distances


def is_wall_near_ball(ball: Ball) -> bool:
    return bool(_wall_distances(ball.position))


def wall_side_for_ball(ball: Ball) -> WallSide | None:
    distances = _wall_distances(ball.position)
    if not distances:
        return None

    nearest_distance = min(distances.values())
    for side in WALL_SIDE_PRIORITY:
        if side in distances and distances[side] <= nearest_distance + WALL_SIDE_TIE_MARGIN_CM:
            return side

    return min(distances, key=distances.get)


def _pickup_candidate_for_side(ball: Ball, side: WallSide) -> tuple[Point, str]:
    x = ball.position.x
    y = ball.position.y
    offset = PICKUP_APPROACH_DISTANCE_CM

    if side == "right":
        return Point(x - offset, y), "RIGHT"
    if side == "left":
        return Point(x + offset, y), "LEFT"
    if side == "bottom":
        return Point(x, y + offset), "DOWN"
    if side == "top":
        return Point(x, y - offset), "UP"

    return Point(x, y), "CENTER"


def _wall_side_for_face_direction(face_direction: str) -> WallSide | None:
    return {
        "RIGHT": "right",
        "LEFT": "left",
        "DOWN": "bottom",
        "UP": "top",
    }.get(face_direction)


def quadrant_order_after_vip(vip_quadrant: Quadrant | None) -> list[Quadrant]:
    order: list[Quadrant] = []

    if vip_quadrant in ALL_QUADRANTS:
        order.append(vip_quadrant)

    for quadrant in QUADRANT_SWEEP_ORDER:
        if quadrant in ALL_QUADRANTS and quadrant not in order:
            order.append(quadrant)

    for quadrant in ALL_QUADRANTS:
        if quadrant not in order:
            order.append(quadrant)

    return order


def _pickup_candidates_for_ball(ball: Ball) -> list[tuple[Point, str]]:
    candidates: list[tuple[Point, str]] = []
    wall_distances = _wall_distances(ball.position)
    primary_side = wall_side_for_ball(ball)

    ordered_sides: list[WallSide] = []
    if primary_side is not None:
        ordered_sides.append(primary_side)

    for side in WALL_SIDE_PRIORITY:
        if side in wall_distances and side not in ordered_sides:
            ordered_sides.append(side)

    for side in ordered_sides:
        candidates.append(_pickup_candidate_for_side(ball, side))

    if not candidates:
        candidates.append((ball.position, "CENTER"))

    return candidates


def generate_pickup_targets(ball: Ball) -> list[PickupTarget]:
    targets = []
    is_wall_pickup = (
        ball.position.x <= EDGE_MARGIN_CM
        or ball.position.x >= FIELD_WIDTH_CM - EDGE_MARGIN_CM
        or ball.position.y <= EDGE_MARGIN_CM
        or ball.position.y >= FIELD_HEIGHT_CM - EDGE_MARGIN_CM
    )

    for raw_point, face_direction in _pickup_candidates_for_ball(ball):
        pickup_point = clamp_point(raw_point, WALL_PICKUP_MARGIN_CM)
        targets.append(
            PickupTarget(
                ball=ball,
                pickup_point=pickup_point,
                face_direction=face_direction,
                is_wall_pickup=is_wall_pickup,
            )
        )

    return targets


def make_pickup_target(ball: Ball) -> PickupTarget:
    targets = generate_pickup_targets(ball)
    primary_side = wall_side_for_ball(ball)

    def target_priority(target: PickupTarget) -> tuple[int, float]:
        target_side = _wall_side_for_face_direction(target.face_direction)
        side_priority = 0 if target_side == primary_side else 1
        return side_priority, distance(ball.position, target.pickup_point)

    return min(targets, key=target_priority)


def make_pickup_targets(balls: list[Ball]) -> list[PickupTarget]:
    return [make_pickup_target(ball) for ball in balls]


def route_length(start_point: Point, targets: list[PickupTarget], goal: Goal) -> float:
    total = 0.0
    current = start_point

    for target in targets:
        total += distance(current, target.pickup_point)
        current = target.pickup_point

    total += distance(current, goal.position)
    return total


def _point_key(point: Point) -> tuple[float, float]:
    return round(point.x, 3), round(point.y, 3)


def _make_segment_cost(cross_obstacle=None, blocked_points: list[Point] | None = None):
    cache: dict[tuple[tuple[float, float], tuple[float, float]], float] = {}

    def segment_cost(start: Point, end: Point) -> float:
        key = (_point_key(start), _point_key(end))
        if key in cache:
            return cache[key]

        try:
            from robot_logic.route_planning.pathfinder import path_length, plan_path

            path = plan_path(
                start,
                end,
                cross_obstacle=cross_obstacle,
                blocked_points=blocked_points,
                blocked_point_radius_cm=BALL_AVOID_RADIUS_CM,
            )
            value = path_length(path) if path else float("inf")
        except ImportError:
            value = distance(start, end)

        cache[key] = value
        return value

    return segment_cost


def _path_between(start: Point, end: Point, cross_obstacle=None) -> list[Point]:
    try:
        from robot_logic.route_planning.pathfinder import plan_path

        path = plan_path(start, end, cross_obstacle=cross_obstacle)
        return path if path else [start, end]
    except ImportError:
        return [start, end]


def _heading_between(start: Point, end: Point) -> float | None:
    if distance(start, end) == 0:
        return None

    return degrees(atan2(end.y - start.y, end.x - start.x))


def _turn_penalty(previous_heading: float | None, next_heading: float | None) -> float:
    if previous_heading is None or next_heading is None:
        return 0.0

    delta = abs((next_heading - previous_heading + 180.0) % 360.0 - 180.0)
    return (delta / 180.0) * TURN_PENALTY_CM


def _wall_cluster_progress(side: WallSide, target: PickupTarget) -> float:
    point = target.ball.position

    if side == "right":
        return point.x
    if side == "left":
        return -point.x
    if side == "bottom":
        return -point.y
    if side == "top":
        return point.y

    return 0.0


def wall_cluster_order_key(target: PickupTarget) -> tuple[int, float, float]:
    side = wall_side_for_ball(target.ball)
    if side is None:
        return 1, 0.0, target.ball.position.x

    return 0, _wall_cluster_progress(side, target), target.ball.position.y


def _wall_cluster_sequence_penalty(previous_target: PickupTarget | None, next_target: PickupTarget) -> float:
    if previous_target is None:
        return 0.0

    previous_side = wall_side_for_ball(previous_target.ball)
    next_side = wall_side_for_ball(next_target.ball)
    if previous_side is None:
        return 0.0

    cluster_distance = distance(previous_target.ball.position, next_target.ball.position)
    if cluster_distance > WALL_CLUSTER_DISTANCE_CM * 1.5:
        return 0.0

    previous_progress = _wall_cluster_progress(previous_side, previous_target)

    if next_side is None:
        next_progress = _wall_cluster_progress(previous_side, next_target)
        if next_progress < previous_progress:
            return WALL_CLUSTER_ORDER_PENALTY_CM + previous_progress - next_progress
        return 0.0

    if previous_side != next_side:
        return 0.0

    next_progress = _wall_cluster_progress(next_side, next_target)
    if next_progress >= previous_progress:
        return 0.0

    return WALL_CLUSTER_ORDER_PENALTY_CM + previous_progress - next_progress


def point_to_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    segment_length_squared = dx * dx + dy * dy

    if segment_length_squared == 0:
        return distance(point, start)

    t = ((point.x - start.x) * dx + (point.y - start.y) * dy) / segment_length_squared
    t = clamp(t, 0.0, 1.0)
    closest = Point(start.x + t * dx, start.y + t * dy)
    return distance(point, closest)


def point_to_path_distance(point: Point, path: list[Point]) -> float:
    if not path:
        return float("inf")

    if len(path) == 1:
        return distance(point, path[0])

    return min(point_to_segment_distance(point, start, end) for start, end in zip(path, path[1:]))


def estimated_route_cost(
    start_point: Point,
    targets: list[PickupTarget],
    goal: Goal,
    cross_obstacle=None,
    blocked_points: list[Point] | None = None,
) -> float:
    segment_cost = _make_segment_cost(cross_obstacle, blocked_points)
    total = 0.0
    current = start_point
    previous_heading = None
    previous_target: PickupTarget | None = None

    for target in targets:
        cost = segment_cost(current, target.pickup_point)
        if isinf(cost):
            return float("inf")

        next_heading = _heading_between(current, target.pickup_point)
        total += (
            cost
            + _turn_penalty(previous_heading, next_heading)
            + _wall_cluster_sequence_penalty(previous_target, target)
        )
        current = target.pickup_point
        previous_heading = next_heading
        previous_target = target

    goal_cost = segment_cost(current, goal.position)
    if isinf(goal_cost):
        return float("inf")

    next_heading = _heading_between(current, goal.position)
    return total + goal_cost + _turn_penalty(previous_heading, next_heading)


def nearest_order(
    start_point: Point,
    targets: list[PickupTarget],
    cross_obstacle=None,
    blocked_points: list[Point] | None = None,
) -> list[PickupTarget]:
    remaining = targets[:]
    ordered: list[PickupTarget] = []
    current = start_point
    segment_cost = _make_segment_cost(cross_obstacle, blocked_points)

    while remaining:
        next_target = min(remaining, key=lambda target: segment_cost(current, target.pickup_point))
        ordered.append(next_target)
        remaining.remove(next_target)
        current = next_target.pickup_point

    return ordered


def exact_order(
    start_point: Point,
    targets: list[PickupTarget],
    goal: Goal,
    cross_obstacle=None,
    blocked_points: list[Point] | None = None,
) -> list[PickupTarget]:
    if len(targets) <= 1:
        return targets[:]

    best_route = targets[:]
    best_cost = float("inf")

    for candidate in permutations(targets):
        candidate_list = list(candidate)
        candidate_cost = estimated_route_cost(start_point, candidate_list, goal, cross_obstacle, blocked_points)
        if candidate_cost < best_cost:
            best_route = candidate_list
            best_cost = candidate_cost

    return best_route


def two_opt_improve(
    start_point: Point,
    targets: list[PickupTarget],
    goal: Goal,
    cross_obstacle=None,
    blocked_points: list[Point] | None = None,
) -> list[PickupTarget]:
    best_route = targets[:]
    improved = True

    while improved:
        improved = False
        best_length = estimated_route_cost(start_point, best_route, goal, cross_obstacle, blocked_points)

        for i in range(len(best_route) - 1):
            for j in range(i + 1, len(best_route)):
                candidate = best_route[:i] + list(reversed(best_route[i:j + 1])) + best_route[j + 1:]
                candidate_length = estimated_route_cost(start_point, candidate, goal, cross_obstacle, blocked_points)

                if candidate_length < best_length:
                    best_route = candidate
                    best_length = candidate_length
                    improved = True

    return best_route


def order_targets(
    start_point: Point,
    targets: list[PickupTarget],
    goal: Goal,
    cross_obstacle=None,
    blocked_points: list[Point] | None = None,
) -> list[PickupTarget]:
    if len(targets) <= MAX_EXACT_ROUTE_TARGETS:
        return exact_order(start_point, targets, goal, cross_obstacle, blocked_points)

    route = nearest_order(start_point, targets, cross_obstacle, blocked_points)
    return two_opt_improve(start_point, route, goal, cross_obstacle, blocked_points)


def apply_on_path_preference(
    start_point: Point,
    ordered_targets: list[PickupTarget],
    cross_obstacle=None,
) -> list[PickupTarget]:
    pending = ordered_targets[:]
    ordered: list[PickupTarget] = []
    current = start_point
    segment_cost = _make_segment_cost(cross_obstacle)

    while pending:
        primary = pending[0]
        path_to_primary = _path_between(current, primary.pickup_point, cross_obstacle)
        candidates = [
            target
            for target in pending[1:]
            if point_to_path_distance(target.ball.position, path_to_primary) <= ON_PATH_BALL_RADIUS_CM
        ]

        if candidates:
            next_target = min(candidates, key=lambda target: segment_cost(current, target.pickup_point))
        else:
            next_target = primary

        ordered.append(next_target)
        pending.remove(next_target)
        current = next_target.pickup_point

    return ordered


def _remove_targets_by_identity(targets: list[PickupTarget], to_remove: list[PickupTarget]) -> list[PickupTarget]:
    remove_ids = {id(target) for target in to_remove}
    return [target for target in targets if id(target) not in remove_ids]


def order_targets_by_quadrant_strategy(
    start_point: Point,
    targets: list[PickupTarget],
    goal: Goal,
    vip_quadrant: Quadrant | None,
    cross_obstacle=None,
) -> list[PickupTarget]:
    if ROUTE_STRATEGY != "vip_quadrant_then_sweep":
        route = order_targets(start_point, targets, goal, cross_obstacle)
        return apply_on_path_preference(start_point, route, cross_obstacle)

    remaining = targets[:]
    route: list[PickupTarget] = []
    current = start_point

    for quadrant in quadrant_order_after_vip(vip_quadrant):
        group = [target for target in remaining if get_quadrant(target.ball.position) == quadrant]
        if not group:
            continue

        group_route = order_targets(current, group, goal, cross_obstacle)
        group_route = apply_on_path_preference(current, group_route, cross_obstacle)
        route.extend(group_route)
        remaining = _remove_targets_by_identity(remaining, group_route)
        current = group_route[-1].pickup_point

    if remaining:
        final_route = order_targets(current, remaining, goal, cross_obstacle)
        final_route = apply_on_path_preference(current, final_route, cross_obstacle)
        route.extend(final_route)

    return route


def route_quadrant_order(route: list[PickupTarget]) -> list[Quadrant]:
    order: list[Quadrant] = []

    for target in route:
        quadrant = get_quadrant(target.ball.position)
        if quadrant not in order:
            order.append(quadrant)

    return order


def choose_route(
    robot_position: Point,
    balls: list[Ball],
    goal_a: Goal,
    cross_obstacle=None,
    keep_vip_first: bool = True,
) -> list[PickupTarget]:
    pickup_targets = make_pickup_targets(balls)

    if not keep_vip_first:
        route = order_targets(robot_position, pickup_targets, goal_a, cross_obstacle)
        return apply_on_path_preference(robot_position, route, cross_obstacle)

    vip_targets = [target for target in pickup_targets if target.ball.is_vip]
    normal_targets = [target for target in pickup_targets if not target.ball.is_vip]

    ordered_route: list[PickupTarget] = []
    current_position = robot_position
    vip_quadrant = None

    if vip_targets:
        white_obstacles = [target.ball.position for target in normal_targets]
        vip_route = order_targets(
            current_position,
            vip_targets,
            goal_a,
            cross_obstacle,
            blocked_points=white_obstacles,
        )
        ordered_route.extend(vip_route)
        current_position = vip_route[-1].pickup_point
        vip_quadrant = get_quadrant(vip_route[-1].ball.position)

    if normal_targets:
        normal_route = order_targets_by_quadrant_strategy(
            current_position,
            normal_targets,
            goal_a,
            vip_quadrant,
            cross_obstacle,
        )
        ordered_route.extend(normal_route)

    return ordered_route


def goal_a() -> Goal:
    return Goal(name="Goal A", position=Point(GOAL_A_X_CM, GOAL_A_Y_CM))


if __name__ == "__main__":
    robot_position = Point(30, 30)
    demo_goal = goal_a()
    balls = [
        Ball(name="Ball 1", position=Point(10, 40), is_vip=False),
        Ball(name="Ball 2", position=Point(80, 30), is_vip=True),
        Ball(name="Ball 3", position=Point(118, 100), is_vip=False),
        Ball(name="Ball 4", position=Point(100, 118), is_vip=False),
        Ball(name="Bottom wall", position=Point(80, 2), is_vip=False),
    ]

    planned_route = choose_route(robot_position, balls, demo_goal)

    print("Planned route:")
    for index, target in enumerate(planned_route, start=1):
        point = target.pickup_point
        print(
            f"{index}. {target.ball.name} | VIP={target.ball.is_vip} | "
            f"pickup=({point.x}, {point.y}) | face={target.face_direction} | "
            f"wall={target.is_wall_pickup}"
        )

    print(f"Quadrant order used: {route_quadrant_order(planned_route)}")
    print(f"End at {demo_goal.name}: ({demo_goal.position.x}, {demo_goal.position.y})")
