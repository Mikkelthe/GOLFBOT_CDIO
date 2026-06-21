import unittest
import math
from utils import Angle

class AngleTestCase(unittest.TestCase):
    def test_radian_conversion(self):
        angle = Angle()
        angle.degrees = 165
        self.assertAlmostEqual(angle.radians, 165*math.pi/180)

    def test_angle_conversion(self):
        angle = Angle()
        angle.radians = 2.63
        self.assertAlmostEqual(angle.degrees, 2.63*180/math.pi)

if __name__ == '__main__':
    unittest.main()
