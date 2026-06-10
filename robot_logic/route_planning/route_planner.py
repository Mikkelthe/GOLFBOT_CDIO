from dataclasses import dataclass
from math import hypot


FIELD_WIDTH_CM = 125.0
FIELD_HEIGHT_CM = 170.0
EDGE_MARGIN_CM = 20
# Vacuum/tube pickup distance is around 14-17 cm; keep this tunable.
PICKUP_OFFSET_CM = 18
SAFE_PICKUP_MARGIN_CM = 3.0
CROSS_SAFETY_RADIUS_CM = 18.0
DETOUR_EXTRA_MARGIN_CM = 12.0


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


def distance(point_a: Point, point_b: Point) -> float:
    return hypot(point_b.x - point_a.x, point_b.y - point_a.y)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def clamp_point(point: Point, margin_cm: float = SAFE_PICKUP_MARGIN_CM) -> Point:
    return Point(
        clamp(point.x, margin_cm, FIELD_WIDTH_CM - margin_cm),
        clamp(point.y, margin_cm, FIELD_HEIGHT_CM - margin_cm),
    )


def make_pickup_target(ball: Ball) -> PickupTarget:
    x = ball.position.x
    y = ball.position.y

    if x <= EDGE_MARGIN_CM:
        pickup_point = clamp_point(Point(x + PICKUP_OFFSET_CM, y))
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="LEFT")

    if x >= FIELD_WIDTH_CM - EDGE_MARGIN_CM:
        pickup_point = clamp_point(Point(x - PICKUP_OFFSET_CM, y))
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="RIGHT")

    if y <= EDGE_MARGIN_CM:
        pickup_point = clamp_point(Point(x, y + PICKUP_OFFSET_CM))
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="DOWN")

    if y >= FIELD_HEIGHT_CM - EDGE_MARGIN_CM:
        pickup_point = clamp_point(Point(x, y - PICKUP_OFFSET_CM))
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="UP")

    return PickupTarget(ball=ball, pickup_point=clamp_point(ball.position), face_direction="CENTER")


def route_length(start_point: Point, targets: list[PickupTarget], goal: Goal) -> float:
    total = 0.0
    current = start_point

    for target in targets:
        total += distance(current, target.pickup_point)
        current = target.pickup_point

    total += distance(current, goal.position)
    return total


def nearest_order(start_point: Point, targets: list[PickupTarget]) -> list[PickupTarget]:
    remaining = targets[:]
    ordered: list[PickupTarget] = []
    current = start_point

    while remaining:
        next_target = min(remaining, key=lambda target: distance(current, target.pickup_point))
        ordered.append(next_target)
        remaining.remove(next_target)
        current = next_target.pickup_point

    return ordered


def two_opt_improve(start_point: Point, targets: list[PickupTarget], goal: Goal) -> list[PickupTarget]:
    best_route = targets[:]
    improved = True

    # Later, A* or obstacle-aware planning can replace this simple ordering step.
    while improved:
        improved = False
        best_length = route_length(start_point, best_route, goal)

        for i in range(len(best_route) - 1):
            for j in range(i + 1, len(best_route)):
                candidate = best_route[:i] + list(reversed(best_route[i:j + 1])) + best_route[j + 1:]
                candidate_length = route_length(start_point, candidate, goal)

                if candidate_length < best_length:
                    best_route = candidate
                    best_length = candidate_length
                    improved = True

    return best_route


def choose_route(robot_position: Point, balls: list[Ball], goal_a: Goal) -> list[PickupTarget]:
    pickup_targets = [make_pickup_target(ball) for ball in balls]

    vip_targets = [target for target in pickup_targets if target.ball.is_vip]
    normal_targets = [target for target in pickup_targets if not target.ball.is_vip]

    ordered_route: list[PickupTarget] = []
    current_position = robot_position

    if vip_targets:
        vip_route = nearest_order(current_position, vip_targets)
        ordered_route.extend(vip_route)
        current_position = vip_route[-1].pickup_point

    if normal_targets:
        normal_route = nearest_order(current_position, normal_targets)
        normal_route = two_opt_improve(current_position, normal_route, goal_a)
        ordered_route.extend(normal_route)

    return ordered_route


def line_crosses_circle(start: Point, end: Point, center: Point, radius_cm: float) -> bool:
    dx = end.x - start.x
    dy = end.y - start.y
    segment_length_squared = dx * dx + dy * dy

    if segment_length_squared == 0:
        return distance(start, center) <= radius_cm

    t = ((center.x - start.x) * dx + (center.y - start.y) * dy) / segment_length_squared
    t = clamp(t, 0.0, 1.0)
    closest = Point(start.x + t * dx, start.y + t * dy)

    return distance(closest, center) <= radius_cm


def choose_detour_point(start: Point, end: Point, center: Point, radius_cm: float) -> Point:
    detour_distance = radius_cm + DETOUR_EXTRA_MARGIN_CM
    diagonal_step = detour_distance * 0.7
    candidates = [
        Point(center.x + detour_distance, center.y),
        Point(center.x - detour_distance, center.y),
        Point(center.x, center.y + detour_distance),
        Point(center.x, center.y - detour_distance),
        Point(center.x + diagonal_step, center.y + diagonal_step),
        Point(center.x - diagonal_step, center.y + diagonal_step),
        Point(center.x + diagonal_step, center.y - diagonal_step),
        Point(center.x - diagonal_step, center.y - diagonal_step),
    ]
    candidates = [clamp_point(candidate) for candidate in candidates]
    candidates = [candidate for candidate in candidates if distance(candidate, center) > radius_cm]

    safe_candidates = [
        candidate
        for candidate in candidates
        if not line_crosses_circle(start, candidate, center, radius_cm)
        and not line_crosses_circle(candidate, end, center, radius_cm)
    ]

    usable_candidates = safe_candidates or candidates
    return min(usable_candidates, key=lambda point: distance(start, point) + distance(point, end))


def add_cross_detours(
    path_points: list[Point],
    cross_center: Point | None,
    safety_radius_cm: float = CROSS_SAFETY_RADIUS_CM,
) -> list[Point]:
    if cross_center is None or len(path_points) < 2:
        return path_points

    detour_path = [path_points[0]]

    for next_point in path_points[1:]:
        current_point = detour_path[-1]

        if line_crosses_circle(current_point, next_point, cross_center, safety_radius_cm):
            detour_point = choose_detour_point(current_point, next_point, cross_center, safety_radius_cm)
            detour_path.append(detour_point)

        detour_path.append(next_point)

    return detour_path


def build_path_points(
    robot_position: Point,
    route: list[PickupTarget],
    goal: Goal,
    cross_center: Point | None = None,
    cross_safety_radius_cm: float = CROSS_SAFETY_RADIUS_CM,
) -> list[Point]:
    # Later, A* can replace this simple detour step.
    path_points = [robot_position] + [target.pickup_point for target in route] + [goal.position]
    return add_cross_detours(path_points, cross_center, cross_safety_radius_cm)


if __name__ == "__main__":
    robot_position = Point(30, 30)
    goal_a = Goal(name="Goal A", position=Point(122, 85))
    balls = [
        Ball(name="Ball 1", position=Point(10, 40), is_vip=False),
        Ball(name="Ball 2", position=Point(80, 30), is_vip=True),
        Ball(name="Ball 3", position=Point(118, 100), is_vip=False),
        Ball(name="Ball 4", position=Point(100, 150), is_vip=False),
    ]

    planned_route = choose_route(robot_position, balls, goal_a)

    print("Planned route:")
    for index, target in enumerate(planned_route, start=1):
        point = target.pickup_point
        print(
            f"{index}. {target.ball.name} | VIP={target.ball.is_vip} | "
            f"pickup=({point.x}, {point.y}) | face={target.face_direction}"
        )

    print(f"End at {goal_a.name}: ({goal_a.position.x}, {goal_a.position.y})")
