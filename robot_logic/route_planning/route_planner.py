from dataclasses import dataclass
from math import hypot


FIELD_WIDTH_CM = 120
FIELD_HEIGHT_CM = 180
EDGE_MARGIN_CM = 20
PICKUP_OFFSET_CM = 18


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


def make_pickup_target(ball: Ball) -> PickupTarget:
    x = ball.position.x
    y = ball.position.y

    if x <= EDGE_MARGIN_CM:
        pickup_point = Point(min(x + PICKUP_OFFSET_CM, FIELD_WIDTH_CM), y)
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="LEFT")

    if x >= FIELD_WIDTH_CM - EDGE_MARGIN_CM:
        pickup_point = Point(max(x - PICKUP_OFFSET_CM, 0), y)
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="RIGHT")

    if y <= EDGE_MARGIN_CM:
        pickup_point = Point(x, min(y + PICKUP_OFFSET_CM, FIELD_HEIGHT_CM))
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="DOWN")

    if y >= FIELD_HEIGHT_CM - EDGE_MARGIN_CM:
        pickup_point = Point(x, max(y - PICKUP_OFFSET_CM, 0))
        return PickupTarget(ball=ball, pickup_point=pickup_point, face_direction="UP")

    return PickupTarget(ball=ball, pickup_point=ball.position, face_direction="CENTER")


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


if __name__ == "__main__":
    robot_position = Point(30, 30)
    goal_a = Goal(name="Goal A", position=Point(170, 60))
    balls = [
        Ball(name="Ball 1", position=Point(10, 40), is_vip=False),
        Ball(name="Ball 2", position=Point(80, 30), is_vip=True),
        Ball(name="Ball 3", position=Point(165, 100), is_vip=False),
        Ball(name="Ball 4", position=Point(100, 112), is_vip=False),
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
