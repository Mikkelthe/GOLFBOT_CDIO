import unittest

from robot_logic.route_planning.pathfinder import (
    WALL_SAFETY_MARGIN_CM,
    path_length,
    plan_path,
)
from robot_logic.route_planning.route_planner import (
    CROSS_SAFETY_RADIUS_CM,
    FIELD_HEIGHT_CM,
    FIELD_WIDTH_CM,
    Point,
    distance,
)


class TestPathfinder(unittest.TestCase):
    def test_path_exists_with_no_obstacle(self):
        path = plan_path(Point(10, 10), Point(110, 150))

        self.assertGreater(len(path), 1)
        self.assertAlmostEqual(path[0].x, 10)
        self.assertAlmostEqual(path[0].y, 10)
        self.assertAlmostEqual(path[-1].x, 110)
        self.assertAlmostEqual(path[-1].y, 150)

    def test_path_stays_inside_field(self):
        path = plan_path(Point(10, 10), Point(110, 150))

        for point in path:
            self.assertGreaterEqual(point.x, WALL_SAFETY_MARGIN_CM)
            self.assertLessEqual(point.x, FIELD_WIDTH_CM - WALL_SAFETY_MARGIN_CM)
            self.assertGreaterEqual(point.y, WALL_SAFETY_MARGIN_CM)
            self.assertLessEqual(point.y, FIELD_HEIGHT_CM - WALL_SAFETY_MARGIN_CM)

    def test_path_avoids_circular_cross_obstacle(self):
        cross = Point(60, 85)
        path = plan_path(Point(10, 85), Point(115, 85), cross)

        self.assertGreater(len(path), 1)
        for point in path:
            self.assertGreater(distance(point, cross), CROSS_SAFETY_RADIUS_CM)

    def test_obstacle_path_is_longer_than_straight_line(self):
        start = Point(10, 85)
        goal = Point(115, 85)
        cross = Point(60, 85)
        path = plan_path(start, goal, cross)

        self.assertGreater(path_length(path), distance(start, goal))

    def test_start_slightly_outside_field_gets_clamped(self):
        path = plan_path(Point(-3, 85), Point(30, 85))

        self.assertGreater(len(path), 1)
        self.assertEqual(path[0].x, WALL_SAFETY_MARGIN_CM)
        self.assertEqual(path[0].y, 85)

    def test_impossible_route_returns_empty_path(self):
        path = plan_path(Point(10, 10), Point(110, 150), cross_center=Point(60, 85), cross_radius_cm=300)

        self.assertEqual(path, [])


if __name__ == "__main__":
    unittest.main()
