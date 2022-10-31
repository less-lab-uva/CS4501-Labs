import unittest
import numpy as np
from astar_class import AStarPlanner

class TestAStar(unittest.TestCase):

    def setUp(self):
        self.astar = AStarPlanner()
        self.testmap = np.zeros((10, 10))
        self.drone_position = [5, 5]

    def test_straight_x(self):
        goal_position = [1, 5]
        trajectory = self.astar.plan(self.testmap, self.drone_position, goal_position)
        expected_trajectory = [[5, 5], [4, 5], [3, 5], [2, 5], [1, 5]]
        self.assertEqual(trajectory, expected_trajectory)

    def test_straight_y(self):
        goal_position = [5, 1]
        trajectory = self.astar.plan(self.testmap, self.drone_position, goal_position)
        expected_trajectory = [[5, 5], [5, 4], [5, 3], [5, 2], [5, 1]]
        self.assertEqual(trajectory, expected_trajectory)

    def test_diagonal(self):
        goal_position = [1, 1]
        trajectory = self.astar.plan(self.testmap, self.drone_position, goal_position)
        expected_trajectory = [[5, 5], [4, 4], [3, 3], [2, 2], [1, 1]]
        self.assertEqual(trajectory, expected_trajectory)

    def test_obstacle(self):
        self.testmap[5][3] = 100
        self.testmap[4][3] = 100
        goal_position = [5, 1]
        trajectory = self.astar.plan(self.testmap, self.drone_position, goal_position)
        expected_trajectory =  [[5, 5], [6, 5], [7, 4], [7, 3], [7, 2], [6, 1], [5, 1]]
        self.assertEqual(trajectory, expected_trajectory)

if __name__ == '__main__':
    unittest.main()