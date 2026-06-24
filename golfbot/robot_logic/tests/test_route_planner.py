import unittest

from robot_logic import RoutePlanner
from utils import Point


class RoutePlannerTestCase(unittest.TestCase):
    def setUp(self):
        self.planner = RoutePlanner()

    def test_plan_best_path_accepts_cross_metadata(self):
        start = Point(300, 500)
        target = Point(1200, 500)
        obstacles = {
            "vertical_box": [(700, 450), (800, 450), (800, 550), (700, 550)],
            "horizontal_box": [(900, 200), (980, 200), (980, 280), (900, 280)],
            "center": [(750, 500)],
        }

        path = self.planner.plan_best_path(start, target, obstacles)

        self.assertGreater(len(path), 2)
        self.assertEqual((path[0].x, path[0].y), (start.x, start.y))
        self.assertEqual((path[-1].x, path[-1].y), (target.x, target.y))

    def test_plan_best_path_rejects_wall_collision_target(self):
        start = Point(300, 300)
        target = Point(50, 150)

        path = self.planner.plan_best_path(start, target)

        self.assertEqual(path, [])

    def test_plan_best_path_rejects_target_inside_wall_clearance(self):
        start = Point(500, 500)
        target = Point(234.6, 500)

        path = self.planner.plan_best_path(start, target)

        self.assertEqual(path, [])

    def test_choose_best_next_ball_skips_unreachable_ball(self):
        robot = Point(300, 300)
        blocked_ball = Point(50, 150)
        reachable_ball = Point(500, 300)

        best_ball = self.planner.choose_best_next_ball(
            robot,
            [blocked_ball, reachable_ball],
        )

        self.assertEqual((best_ball.x, best_ball.y), (reachable_ball.x, reachable_ball.y))
