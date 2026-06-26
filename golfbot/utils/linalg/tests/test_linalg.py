import unittest
import numpy as np
from utils.linalg import Vector2
from utils import Angle

class VectorTests(unittest.TestCase):
    def test_addition(self):
        vec1 = Vector2(1, 2)
        vec2 = Vector2(3, 4)
        self.assertEqual(vec1 + vec2, Vector2(4, 6))

    def test_subtraction(self):
        vec1 = Vector2(1, 2)
        vec2 = Vector2(3, 4)
        self.assertEqual(vec1 - vec2, Vector2(-2, -2))

    def test_multiplication(self):
        vec1 = Vector2(1, 2)
        scalar = 2
        self.assertEqual(vec1 * scalar, Vector2(2, 4))

    def test_negation(self):
        vec = Vector2(-1, 2)
        self.assertEqual(-vec, Vector2(1, -2))

    def test_len(self):
        vec = Vector2(0, 0)
        self.assertEqual(len(vec), 2)

    def test_getitem(self):
        vec = Vector2(1, 2)
        self.assertEqual((vec[0], vec[1]), (1, 2))

    def test_eq(self):
        vec = Vector2(1, 2)

        self.assertEqual(vec, (1, 2))
        self.assertEqual(vec, [1, 2])
        self.assertEqual(vec, np.array([1, 2]))
        self.assertEqual(vec, Vector2(1, 2))

    def test_rotate(self):
        vec = Vector2(0, 1)
        angle = Angle()
        angle.radians = np.pi / 4
        rotated = vec.rotate(angle)
        expected = Vector2(-1 / np.sqrt(2), 1 / np.sqrt(2))
        diff_sum = np.sum(np.abs(rotated - expected))

        self.assertAlmostEqual(diff_sum, 0)

    def test_dot(self):
        vec1 = Vector2(1, 2)
        vec2 = Vector2(3, 4)

        self.assertEqual(Vector2.dot(vec1, vec2), 11)

    def test_project(self):
        vec1 = Vector2(1, 0)
        vec2 = Vector2(2, 2)
        self.assertEqual(Vector2.project(vec1, vec2), (0.5, 0.5))

    def test_unsignedAngle(self):
        vec1 = Vector2(0, 1)
        vec2 = Vector2(1, 1)
        angle = Vector2.unsignedAngle(vec1, vec2)
        expected = np.pi / 4

        self.assertAlmostEqual(angle, expected)

    def test_signedAngle(self):
        vec1 = Vector2(0, 1)
        vec2 = Vector2(1, 1)
        angle = Vector2.signedAngle(vec1, vec2)
        expected = -np.pi / 4

        self.assertAlmostEqual(angle, expected)

    def test_x_property(self):
        vec = Vector2(1, 2)
        with self.assertRaises(AttributeError):
            vec.x = 2

        self.assertEqual(vec.x, 1)

    def test_y_property(self):
        vec = Vector2(1, 2)
        with self.assertRaises(AttributeError):
            vec.y = 3

        self.assertEqual(vec.y, 2)

    def test_magnitude(self):
        vec = Vector2(1, -1)

        self.assertAlmostEqual(vec.magnitude, np.sqrt(2))

    def test_normalized(self):
        vec = Vector2(1, -1)
        normalized = vec.normalized
        expected = Vector2(1 / np.sqrt(2), -1 / np.sqrt(2))
        diff_sum = np.sum(np.abs(normalized - expected))
        self.assertAlmostEqual(diff_sum, 0)

if __name__ == '__main__':
    unittest.main()
