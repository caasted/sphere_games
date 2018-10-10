import unittest
import host.setup_arena
from geometry_msgs.msg import Point
import host.utilities as util
import host.constants as constants

class MyTestCase(unittest.TestCase):

    def test_px2mm(self):

        input = constants.ORIGIN_PIXELS
        expected = Point(0, 0, 0)

        # Act
        actual = util.pixels_2_mm(input)

        # Assert
        self.assertEqual(expected, actual)

    def test_px2mm_100_100(self):

        input = Point(int(constants.ORIGIN_PIXELS.x + (100 * constants.COVERT_MM2PIXEL)),
                      int(constants.ORIGIN_PIXELS.y - (100 * constants.COVERT_MM2PIXEL)),
                      0)
        expected = Point(100 , 100, 0)

        # Act
        actual = util.pixels_2_mm(input)

        # Assert
        self.assertAlmostEqual(expected.x, actual.x,delta=1)
        self.assertAlmostEqual(expected.y, actual.y, delta=1)

    def test_px2mm_100_neg100(self):

        input = Point(int(constants.ORIGIN_PIXELS.x + (100 * constants.COVERT_MM2PIXEL)),
                      int(constants.ORIGIN_PIXELS.y + (100 * constants.COVERT_MM2PIXEL)),
                      0)
        expected = Point(100 , -100, 0)

        # Act
        actual = util.pixels_2_mm(input)

        # Assert
        self.assertAlmostEqual(expected.x, actual.x,delta=1)
        self.assertAlmostEqual(expected.y, actual.y, delta=1)

    def test_mm2px(self):

        expected = constants.ORIGIN_PIXELS
        input = Point(0, 0, 0)

        # Act
        actual = util.mm_2_pixel(input)

        # Assert
        self.assertEqual(expected, actual)

    def test_mm2px_100_100(self):
        expected = Point(int(constants.ORIGIN_PIXELS.x + (100 * constants.COVERT_MM2PIXEL)),
                         int(constants.ORIGIN_PIXELS.y - (100 * constants.COVERT_MM2PIXEL)),
                         0)
        input = Point(100, 100, 0)

        # Act
        actual = util.mm_2_pixel(input)

        # Assert
        self.assertEqual(expected, actual)

    def test_mm2px_100_neg100(self):
        expected = Point(int(constants.ORIGIN_PIXELS.x + (100 * constants.COVERT_MM2PIXEL)),
                         int(constants.ORIGIN_PIXELS.y + (100 * constants.COVERT_MM2PIXEL)),
                         0)
        input = Point(100, -100, 0)

        # Act
        actual = util.mm_2_pixel(input)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading(self):

        # Arrange
        p1 = Point(0,0,0)
        p2 = Point(4,3,0)
        expected = 53.13010235415598

        # Act
        actual = util.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles1(self):
        # Arrange
        p1 = Point(0, 0, 0)
        p2 = Point(3, 3, 0)
        expected = 45.0

        # Act
        actual = util.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles2(self):
        # Arrange
        p1 = Point(0, 0, 0)
        p2 = Point(3, -3, 0)
        expected = 135.0

        # Act
        actual = util.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles3(self):
        # Arrange
        p1 = Point(0, 0, 0)
        p2 = Point(-3, -3, 0)
        expected = -135

        # Act
        actual = util.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles4(self):
        # Arrange
        p1 = Point(0, 0, 0)
        p2 = Point(-3, 3, 0)
        expected = -45

        # Act
        actual = util.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)



    def test_calculate_distance_positive(self):

        # Arrange
        p1 = Point(0,0,0)
        p2 = Point(4,3,0)
        expected = 5.0

        # Act
        actual = util.calculate_distance(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_distance_negative(self):

        # Arrange
        p1 = Point(0,0,0)
        p2 = Point(4,-3,0)
        expected = 5.0

        # Act
        actual = util.calculate_distance(p1, p2)

        # Assert
        self.assertEqual(expected, actual)


if __name__ == '__main__':
    unittest.main()
