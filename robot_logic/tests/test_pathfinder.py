import unittest

from robot_logic.navigation_config import (
    BALL_AVOID_RADIUS_CM,
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
)
from robot_logic.route_planning.obstacles import AxisAlignedBox, CrossObstacle, straight_line_crosses_obstacle
from robot_logic.route_planning.pathfinder import (
    WALL_SAFETY_MARGIN_CM,
    path_length,
    plan_path,
    smooth_path,
)
from robot_logic.route_planning.route_planner import (
    Ball,
    Goal,
    Point,
    choose_route,
    distance,
    get_quadrant,
    make_pickup_target,
    quadrant_order_after_vip,
    wall_side_for_ball,
)


def count_turns(path: list[Point]) -> int:
    directions = []

    for start, end in zip(path, path[1:]):
        dx = end.x - start.x
        dy = end.y - start.y
        if abs(dx) < 0.01 and abs(dy) < 0.01:
            continue

        length = distance(start, end)
        directions.append((round(dx / length, 2), round(dy / length, 2)))

    return sum(1 for previous, current in zip(directions, directions[1:]) if previous != current)


class TestPathfinder(unittest.TestCase):
    def test_path_exists_with_no_obstacle(self):
        goal = Point(110, FIELD_HEIGHT_CM - WALL_SAFETY_MARGIN_CM)
        path = plan_path(Point(10, 10), goal)

        self.assertGreater(len(path), 1)
        self.assertAlmostEqual(path[0].x, 10)
        self.assertAlmostEqual(path[0].y, 10)
        self.assertAlmostEqual(path[-1].x, goal.x)
        self.assertAlmostEqual(path[-1].y, goal.y)

    def test_path_stays_inside_field(self):
        path = plan_path(Point(10, 10), Point(110, FIELD_HEIGHT_CM + 25))

        for point in path:
            self.assertGreaterEqual(point.x, WALL_SAFETY_MARGIN_CM)
            self.assertLessEqual(point.x, FIELD_WIDTH_CM - WALL_SAFETY_MARGIN_CM)
            self.assertGreaterEqual(point.y, WALL_SAFETY_MARGIN_CM)
            self.assertLessEqual(point.y, FIELD_HEIGHT_CM - WALL_SAFETY_MARGIN_CM)

    def test_smooth_path_has_fewer_or_equal_points_and_keeps_endpoints(self):
        raw_path = plan_path(Point(10, 10), Point(130, 100))
        simplified_path = smooth_path(raw_path)

        self.assertLessEqual(len(simplified_path), len(raw_path))
        self.assertEqual(simplified_path[0], raw_path[0])
        self.assertEqual(simplified_path[-1], raw_path[-1])

    def test_smooth_path_does_not_cross_cross_arm_obstacle(self):
        cross = CrossObstacle(
            arms=(
                AxisAlignedBox(min_x=55, max_x=65, min_y=30, max_y=95),
                AxisAlignedBox(min_x=35, max_x=85, min_y=58, max_y=68),
            ),
            safety_margin_cm=3.0,
        )

        raw_path = plan_path(Point(10, 62), Point(130, 62), cross_obstacle=cross)
        simplified_path = smooth_path(raw_path, cross_obstacle=cross)

        self.assertLessEqual(len(simplified_path), len(raw_path))
        for start, end in zip(simplified_path, simplified_path[1:]):
            self.assertFalse(straight_line_crosses_obstacle(start, end, cross))

    def test_smooth_path_has_no_more_turns_than_raw_path(self):
        cross = CrossObstacle(
            arms=(AxisAlignedBox(min_x=55, max_x=65, min_y=45, max_y=85),),
            safety_margin_cm=3.0,
        )

        raw_path = plan_path(Point(20, 65), Point(120, 65), cross_obstacle=cross)
        simplified_path = smooth_path(raw_path, cross_obstacle=cross)

        self.assertLessEqual(count_turns(simplified_path), count_turns(raw_path))

    def test_field_dimensions_remain_canonical(self):
        self.assertEqual(FIELD_WIDTH_CM, 170.0)
        self.assertEqual(FIELD_HEIGHT_CM, 125.0)

    def test_path_avoids_cross_shape_obstacle(self):
        cross = CrossObstacle(
            arms=(
                AxisAlignedBox(min_x=55, max_x=65, min_y=30, max_y=95),
                AxisAlignedBox(min_x=35, max_x=85, min_y=58, max_y=68),
            ),
            fallback_center=Point(60, 62),
            safety_margin_cm=3.0,
        )

        path = plan_path(Point(10, 62), Point(130, 62), cross_obstacle=cross)

        self.assertGreater(len(path), 1)
        for point in path:
            self.assertFalse(cross.contains(point))

    def test_near_cross_point_outside_arms_is_not_blocked_by_big_circle(self):
        cross = CrossObstacle(
            arms=(
                AxisAlignedBox(min_x=55, max_x=65, min_y=30, max_y=50),
                AxisAlignedBox(min_x=70, max_x=90, min_y=60, max_y=70),
            ),
            fallback_center=Point(62, 52),
            safety_margin_cm=2.0,
        )

        self.assertFalse(cross.contains(Point(67.5, 55)))

    def test_fallback_center_obstacle_is_small(self):
        cross = CrossObstacle(fallback_center=Point(60, 62), fallback_radius_cm=8.0)

        self.assertTrue(cross.contains(Point(64, 62)))
        self.assertFalse(cross.contains(Point(70, 62)))

    def test_obstacle_path_is_longer_than_straight_line(self):
        start = Point(10, 62)
        goal = Point(115, 62)
        cross = CrossObstacle(
            arms=(AxisAlignedBox(min_x=55, max_x=65, min_y=45, max_y=80),),
            safety_margin_cm=3.0,
        )
        path = plan_path(start, goal, cross_obstacle=cross)

        self.assertGreater(path_length(path), distance(start, goal))

    def test_start_slightly_outside_field_gets_clamped(self):
        path = plan_path(Point(-3, 85), Point(30, 85))

        self.assertGreater(len(path), 1)
        self.assertEqual(path[0].x, WALL_SAFETY_MARGIN_CM)
        self.assertEqual(path[0].y, 85)

    def test_all_input_balls_are_included_in_route(self):
        robot_position = Point(20, 20)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="vip", position=Point(130, 100), is_vip=True),
            Ball(name="bottom", position=Point(80, 2)),
            Ball(name="left", position=Point(2, 70)),
            Ball(name="white", position=Point(90, 80)),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertCountEqual([target.ball.name for target in route], [ball.name for ball in balls])

    def test_wall_near_balls_are_still_included_in_route(self):
        robot_position = Point(20, 20)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="vip", position=Point(130, 100), is_vip=True),
            Ball(name="right_wall", position=Point(FIELD_WIDTH_CM - 3, 35)),
            Ball(name="bottom_wall", position=Point(80, 3)),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertCountEqual([target.ball.name for target in route], [ball.name for ball in balls])

    def test_bottom_wall_ball_gets_pickup_target(self):
        target = make_pickup_target(Ball(name="bottom", position=Point(80, 2)))

        self.assertEqual(target.face_direction, "DOWN")
        self.assertGreaterEqual(target.pickup_point.y, 2)
        self.assertGreaterEqual(target.pickup_point.y, WALL_SAFETY_MARGIN_CM)

    def test_right_wall_pickup_target_approaches_from_left_inside_field(self):
        ball = Ball(name="right", position=Point(FIELD_WIDTH_CM - 4, 30))
        target = make_pickup_target(ball)

        self.assertEqual(wall_side_for_ball(ball), "right")
        self.assertEqual(target.face_direction, "RIGHT")
        self.assertLess(target.pickup_point.x, ball.position.x)
        self.assertGreaterEqual(target.pickup_point.x, WALL_SAFETY_MARGIN_CM)
        self.assertLessEqual(target.pickup_point.x, FIELD_WIDTH_CM - WALL_SAFETY_MARGIN_CM)

    def test_bottom_right_corner_prefers_right_wall_approach_when_side_wall_is_close(self):
        target = make_pickup_target(Ball(name="corner", position=Point(FIELD_WIDTH_CM - 5, 2)))

        self.assertEqual(target.face_direction, "RIGHT")

    def test_right_wall_cluster_prefers_inside_to_wall_order(self):
        robot_position = Point(40, 40)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="vip", position=Point(100, 100), is_vip=True),
            Ball(name="outer_right", position=Point(FIELD_WIDTH_CM - 3, 30)),
            Ball(name="inner_right", position=Point(FIELD_WIDTH_CM - 14, 30)),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertEqual([target.ball.name for target in route], ["vip", "inner_right", "outer_right"])

    def test_right_wall_cluster_prefers_nearby_interior_ball_before_wall_pair(self):
        robot_position = Point(40, 40)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="vip", position=Point(100, 100), is_vip=True),
            Ball(name="interior", position=Point(FIELD_WIDTH_CM - 34, 30)),
            Ball(name="inner_right", position=Point(FIELD_WIDTH_CM - 14, 30)),
            Ball(name="outer_right", position=Point(FIELD_WIDTH_CM - 3, 30)),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertEqual(
            [target.ball.name for target in route],
            ["vip", "interior", "inner_right", "outer_right"],
        )

    def test_vip_is_first_even_when_white_ball_is_closer(self):
        robot_position = Point(20, 20)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="near_white", position=Point(25, 22), is_vip=False),
            Ball(name="vip", position=Point(130, 100), is_vip=True),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertEqual(route[0].ball.name, "vip")

    def test_path_to_vip_avoids_white_ball_obstacle(self):
        robot_position = Point(20, 60)
        vip = Ball(name="vip", position=Point(130, 60), is_vip=True)
        white = Ball(name="white", position=Point(75, 60), is_vip=False)
        vip_target = make_pickup_target(vip)

        path = plan_path(robot_position, vip_target.pickup_point, blocked_points=[white.position])

        self.assertGreater(len(path), 1)
        for point in path:
            self.assertGreater(distance(point, white.position), BALL_AVOID_RADIUS_CM)

    def test_after_vip_on_the_way_ball_is_collected_before_far_ball(self):
        robot_position = Point(20, 20)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="vip", position=Point(100, 100), is_vip=True),
            Ball(name="far", position=Point(150, 100)),
            Ball(name="on_way", position=Point(125, 100)),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertEqual([target.ball.name for target in route], ["vip", "on_way", "far"])

    def test_quadrant_helper_returns_expected_quadrants(self):
        self.assertEqual(get_quadrant(Point(10, 100)), "top_left")
        self.assertEqual(get_quadrant(Point(140, 100)), "top_right")
        self.assertEqual(get_quadrant(Point(10, 20)), "bottom_left")
        self.assertEqual(get_quadrant(Point(140, 20)), "bottom_right")

    def test_quadrant_strategy_orders_targets_after_vip_quadrant(self):
        robot_position = Point(20, 20)
        goal = Goal(name="Goal A", position=Point(FIELD_WIDTH_CM - 8, 64.5))
        balls = [
            Ball(name="vip", position=Point(140, 100), is_vip=True),
            Ball(name="tr", position=Point(130, 90)),
            Ball(name="tl", position=Point(30, 90)),
            Ball(name="bl", position=Point(30, 20)),
            Ball(name="br", position=Point(140, 20)),
        ]

        route = choose_route(robot_position, balls, goal)

        self.assertEqual([target.ball.name for target in route], ["vip", "tr", "tl", "bl", "br"])
        self.assertEqual(quadrant_order_after_vip("top_right"), ["top_right", "top_left", "bottom_left", "bottom_right"])

    def test_no_circular_import_failures(self):
        __import__("robot_logic.navigation_config")
        __import__("robot_logic.route_planning.route_planner")
        __import__("robot_logic.route_planning.pathfinder")
        __import__("robot_logic.route_planning.obstacles")
        __import__("robot_logic.robot_detection.aruco_robot_detector")


if __name__ == "__main__":
    unittest.main()
