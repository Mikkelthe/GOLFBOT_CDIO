import unittest
from utils import Point

class PointTestCase(unittest.TestCase):
    def test_getitem(self):
        point = Point(1, 2)
        self.assertEqual((point[0], point[1]), (1, 2))

    def test_getitem_error(self):
        self.assertRaises(IndexError, lambda: Point(1, 2)[2])

    def test_rounding(self):
        point = Point(1.4, 2.6)
        self.assertEqual((point[0], point[1]), (1, 3))
if __name__ == '__main__':
    unittest.main()
