import unittest

from robot_logic import RoutePlanner
from robot_logic._navigation_config import ROBOT_RADIUS_CM
from utils import Point
from utils.settings import court_settings


class RoutePlannerTestCase(unittest.TestCase):
    def setUp(self):
        self.planner = RoutePlanner()

    @staticmethod
    def _safe_target_bounds_px():
        court_width_px = court_settings.image_width - 2 * court_settings.padding
        court_height_px = court_settings.image_height - 2 * court_settings.padding
        robot_radius_x_px = ROBOT_RADIUS_CM * court_width_px / court_settings.court_width
        robot_radius_y_px = ROBOT_RADIUS_CM * court_height_px / court_settings.court_height
        wall_inset_px = court_settings.padding + court_settings.wall_thickness
        return (
            wall_inset_px + robot_radius_x_px + court_settings.wall_clearance_extra_px,
            court_settings.image_width - wall_inset_px - robot_radius_x_px - court_settings.wall_clearance_extra_px,
            wall_inset_px + robot_radius_y_px + court_settings.wall_clearance_extra_px,
            court_settings.image_height - wall_inset_px - robot_radius_y_px - court_settings.wall_clearance_extra_px,
        )

    @classmethod
    def _project_target_to_safe_bounds(cls, point: Point) -> Point:
        min_x, max_x, min_y, max_y = cls._safe_target_bounds_px()
        return Point(
            min(max(point.x, min_x), max_x),
            min(max(point.y, min_y), max_y),
        )

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

    def test_plan_best_path_projects_wall_collision_target_to_safe_boundary(self):
        start = Point(300, 300)
        target = Point(50, 150)

        path = self.planner.plan_best_path(start, target)

        projected_target = self._project_target_to_safe_bounds(target)

        self.assertGreaterEqual(len(path), 2)
        self.assertEqual((path[0].x, path[0].y), (start.x, start.y))
        self.assertEqual((path[-1].x, path[-1].y), (projected_target.x, projected_target.y))

    def test_plan_best_path_projects_target_inside_wall_clearance(self):
        start = Point(500, 500)
        target = Point(234.6, 500)

        path = self.planner.plan_best_path(start, target)
        projected_target = self._project_target_to_safe_bounds(target)

        self.assertGreaterEqual(len(path), 2)
        self.assertEqual((path[-1].x, path[-1].y), (projected_target.x, projected_target.y))

    def test_plan_best_path_projects_target_inside_extra_wall_clearance(self):
        start = Point(500, 500)
        min_x, _, _, _ = self._safe_target_bounds_px()
        target = Point(min_x - 1, 500)

        path = self.planner.plan_best_path(start, target)
        projected_target = self._project_target_to_safe_bounds(target)

        self.assertGreaterEqual(len(path), 2)
        self.assertEqual((path[-1].x, path[-1].y), (projected_target.x, projected_target.y))

    def test_choose_best_next_ball_skips_unreachable_ball(self):
        robot = Point(300, 300)
        blocked_ball = Point(500, 300)
        reachable_ball = Point(300, 500)
        obstacles = [
            ((450, 250), (550, 250), (550, 350), (450, 350)),
        ]

        best_ball = self.planner.choose_best_next_ball(
            robot,
            [blocked_ball, reachable_ball],
            obstacles=obstacles,
        )

        self.assertEqual((best_ball.x, best_ball.y), (reachable_ball.x, reachable_ball.y))

    def test_choose_best_next_ball_can_pick_ball_outside_safe_bounds(self):
        robot = Point(300, 300)
        blocked_ball = Point(50, 150)
        reachable_ball = Point(500, 300)

        best_ball = self.planner.choose_best_next_ball(
            robot,
            [blocked_ball, reachable_ball],
        )

        self.assertEqual((best_ball.x, best_ball.y), (blocked_ball.x, blocked_ball.y))
