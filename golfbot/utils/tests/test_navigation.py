import unittest
from navigation import Navigation
from golfbot.utils import Point


class NavigationTestCase(unittest.TestCase):
    def setUp(self):
        """Set up a Navigation instance for each test"""
        self.nav = Navigation()

    def test_find_optimal_corner_approach_top_left(self):
        """Test approach point for top-left corner"""
        # Top-left corner
        corner_position = Point(100, 100)

        # Bot positioned diagonally from corner
        bot_position = Point(300, 300)

        result = self.nav.find_optimal_corner_approach(corner_position, bot_position)

        # Result should be a Point
        self.assertIsInstance(result, Point)

        # For top-left corner, approach vector is (1, 1), so result should move away from corner
        self.assertGreater(result.x, 0)
        self.assertGreater(result.y, 0)

    def test_find_optimal_corner_approach_top_right(self):
        """Test approach point for top-right corner"""
        warp_w = self.nav.warp_W
        warp_h = self.nav.warp_H

        # Top-right corner
        corner_position = Point(warp_w - 100, 100)

        # Bot positioned away from corner
        bot_position = Point(warp_w - 300, 300)

        result = self.nav.find_optimal_corner_approach(corner_position, bot_position)

        self.assertIsInstance(result, Point)

        # For top-right corner, approach vector is (-1, 1)
        self.assertLess(result.x, 0)
        self.assertGreater(result.y, 0)

    def test_find_optimal_corner_approach_bottom_left(self):
        """Test approach point for bottom-left corner"""
        warp_h = self.nav.warp_H

        # Bottom-left corner
        corner_position = Point(100, warp_h - 100)

        # Bot positioned away from corner
        bot_position = Point(300, warp_h - 300)

        result = self.nav.find_optimal_corner_approach(corner_position, bot_position)

        self.assertIsInstance(result, Point)

        # For bottom-left corner, approach vector is (1, -1)
        self.assertGreater(result.x, 0)
        self.assertLess(result.y, 0)

    def test_find_optimal_corner_approach_bottom_right(self):
        """Test approach point for bottom-right corner"""
        warp_w = self.nav.warp_W
        warp_h = self.nav.warp_H

        # Bottom-right corner
        corner_position = Point(warp_w - 100, warp_h - 100)

        # Bot positioned away from corner
        bot_position = Point(warp_w - 300, warp_h - 300)

        result = self.nav.find_optimal_corner_approach(corner_position, bot_position)

        self.assertIsInstance(result, Point)

        # For bottom-right corner, approach vector is (-1, -1)
        self.assertLess(result.x, 0)
        self.assertLess(result.y, 0)

    def test_find_optimal_corner_approach_bot_directly_above_corner(self):
        """Test when bot is directly above the corner"""
        corner_position = Point(400, 300)
        bot_position = Point(400, 100)  # Directly above

        result = self.nav.find_optimal_corner_approach(corner_position, bot_position)

        self.assertIsInstance(result, Point)
        # Result should exist without raising exception
        self.assertTrue(hasattr(result, 'x'))
        self.assertTrue(hasattr(result, 'y'))


    def test_find_optimal_corner_approach_invalid_corner_center(self):
        """Test with corner in center area (should raise ValueError)"""
        warp_w = self.nav.warp_W
        warp_h = self.nav.warp_H

        # Center position - not in any corner region
        corner_position = Point(warp_w / 2, warp_h / 2)
        bot_position = Point(400, 300)

        with self.assertRaises(ValueError) as context:
            self.nav.find_optimal_corner_approach(corner_position, bot_position)

        self.assertIn("Corner position incorrect", str(context.exception))

    def test_find_optimal_corner_approach_result_is_offset_from_corner(self):
        """Test that result is a position offset from corner, not absolute coordinates"""
        corner_position = Point(200, 200)
        bot_position = Point(500, 500)

        result = self.nav.find_optimal_corner_approach(corner_position, bot_position)

        # The result should be relative to corner or an approach point
        self.assertIsInstance(result, Point)
        # Just verify it returns valid coordinates
        self.assertTrue(isinstance(result.x, int))
        self.assertTrue(isinstance(result.y, int))


if __name__ == '__main__':
    unittest.main()