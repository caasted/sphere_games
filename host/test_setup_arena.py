import unittest
import setup_arena
from geometry_msgs.msg import Point

class MyTestCase(unittest.TestCase):

    def test_calculate_error_heading(self):

        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0,0,0)
        p2 = Point(4,3,0)
        expected = 53.13010235415598

        # Act
        actual = a.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles1(self):
        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0, 0, 0)
        p2 = Point(3, 3, 0)
        expected = 45.0

        # Act
        actual = a.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles2(self):
        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0, 0, 0)
        p2 = Point(3, -3, 0)
        expected = 135.0

        # Act
        actual = a.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles3(self):
        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0, 0, 0)
        p2 = Point(-3, -3, 0)
        expected = -135

        # Act
        actual = a.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_error_heading_isoceles4(self):
        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0, 0, 0)
        p2 = Point(-3, 3, 0)
        expected = -45

        # Act
        actual = a.calculate_error_heading(p1, p2)

        # Assert
        self.assertEqual(expected, actual)



    def test_calculate_distance_positive(self):

        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0,0,0)
        p2 = Point(4,3,0)
        expected = 5.0

        # Act
        actual = a.calculate_distance(p1, p2)

        # Assert
        self.assertEqual(expected, actual)

    def test_calculate_distance_negative(self):

        # Arrange
        a = setup_arena.ArenaSetup()
        p1 = Point(0,0,0)
        p2 = Point(4,-3,0)
        expected = 5.0

        # Act
        actual = a.calculate_distance(p1, p2)

        # Assert
        self.assertEqual(expected, actual)


if __name__ == '__main__':
    unittest.main()
