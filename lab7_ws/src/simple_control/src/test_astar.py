import unittest
import numpy as np
import matplotlib.pyplot as plt
from astar_class import AStarPlanner

class TestAStar(unittest.TestCase):

    def setUp(self):
        self.astar = AStarPlanner(safe_distance=1)
        self.testmap = np.zeros((10, 10))
        self.testmap[9, 9] = 255
        self.drone_position = [5, 5]

    def test_straight_x(self):
        goal_position = [1, 5]
        self.path = self.astar.plan(self.testmap, self.drone_position, goal_position)
        self.expected_path = [[5, 5], [4, 5], [3, 5], [2, 5], [1, 5]]
        self.assertEqual(self.path, self.expected_path)

    def test_straight_y(self):
        goal_position = [5, 1]
        self.path = self.astar.plan(self.testmap, self.drone_position, goal_position)
        self.expected_path = [[5, 5], [5, 4], [5, 3], [5, 2], [5, 1]]
        self.assertEqual(self.path, self.expected_path)

    def test_diagonal(self):
        goal_position = [1, 1]
        self.path = self.astar.plan(self.testmap, self.drone_position, goal_position)
        self.expected_path = [[5, 5], [4, 4], [3, 3], [2, 2], [1, 1]]
        self.assertEqual(self.path, self.expected_path)

    def test_obstacle(self):
        self.testmap[5][3] = 255
        self.testmap[4][3] = 255
        goal_position = [5, 1]
        self.path = self.astar.plan(self.testmap, self.drone_position, goal_position)
        self.expected_path =  [[5, 5], [6, 5], [7, 4], [7, 3], [7, 2], [6, 1], [5, 1]]
        self.assertEqual(self.path, self.expected_path)

    def tearDown(self):
        self.visualize_test(self.testmap, self.path, self.expected_path)

    def visualize_test(self, grid_map, computed_path, expected_path):
        plt.figure(self._testMethodName)
        # Plot the grid
        plt.imshow(grid_map, cmap='Greys')
        # Plot computed path
        x = np.array(computed_path)[:, 1]
        y = np.array(computed_path)[:, 0]
        plt.scatter(x, y)
        plt.plot(x, y, linestyle="--", label="Computed path")
        # Plot expected path
        x = np.array(expected_path)[:, 1]
        y = np.array(expected_path)[:, 0]
        plt.scatter(x, y)
        plt.plot(x, y, linestyle="--", label="Expected path")
        # Display plot
        plt.title(self._testMethodName)
        plt.legend()
        plt.show()

    @classmethod 
    def tearDownClass(self):
        pass

if __name__ == '__main__':
    unittest.main()
