from Queue import PriorityQueue
import copy
import math


class AStarPlanner:

  def __init__(self, safe_distance=1):
    self.safe_distance = safe_distance

  def plan(self, map_data, drone_position, goal_position):
    self.validate_data(map_data, drone_position, goal_position)
    map_data = self.expand_obstacles(map_data, self.safe_distance)
    frontier = PriorityQueue()
    frontier.put((0, drone_position))
    came_from = {}
    cost_so_far = {}
    came_from[str(drone_position)] = None
    cost_so_far[str(drone_position)] = 0
    final_path = []
    while not frontier.empty():
      current = frontier.get()
      current = list(current[1])
      if current[0] == goal_position[0] and current[1] == goal_position[1]:
          while came_from[str(current)] != None:
              final_path.append(current)
              current = came_from[str(current)]
          final_path.append(current)
          return list(reversed(final_path))
      for next in self.get_neighbors(current, map_data):
        edge_cost = math.sqrt(((current[0] - next[0])**2) + ((current[1] - next[1]) ** 2))
        goal_cost = math.sqrt(((goal_position[0] - next[0])**2) + ((goal_position[1] - next[1]) ** 2))
        new_cost = cost_so_far[str(current)] + edge_cost
        if str(next) not in cost_so_far or new_cost < cost_so_far[str(next)]:
            cost_so_far[str(next)] = new_cost
            priority = new_cost + goal_cost
            frontier.put((priority, next))
            came_from[str(next)] = current

    print("Error: No Path Found")
    return [drone_position]

  # Get the children nodes for the current node
  def get_neighbors(self, node_in, map_in):
    # A list of neighbors
    neighbors = []
    # Get the current position
    pos = node_in
    # For all adjacent values
    for x_dim in range(-1, 2):
      for y_dim in range(-1, 2):
        # If the values are not 0, 0 (which is itself)
        if not (x_dim == 0 and y_dim == 0):
          # If the values are inside the map
          if (0 <= pos[0] + x_dim < map_in.shape[0]) and(0 <= pos[1] + y_dim < map_in.shape[1]):
            # If that value is an open square on the map
            if map_in[pos[0] + x_dim][pos[1] + y_dim] == 0:
              # Create a neighbor with the new position
              n = [pos[0] + x_dim, pos[1] + y_dim]
              # Save the neighbors
              neighbors.append(n)
    return neighbors

  # Validate the incoming data
  def validate_data(self, map_data, drone_position, goal_position):
    # Confirm that the map has two dimensions
    assert(len(map_data.shape) == 2)
    # Confirm that the drone and goal position are two points
    assert(len(drone_position) == 2)
    assert(len(goal_position) == 2)
    # Confirm that all positions are integers
    for x in drone_position + goal_position:
      assert(type(x) == int)
    # Confirm the both the start and end goal lie inside the map
    assert(0 <= drone_position[0] < map_data.shape[0])
    assert(0 <= drone_position[1] < map_data.shape[1])
    assert(0 <= goal_position[0] < map_data.shape[0])
    assert(0 <= goal_position[1] < map_data.shape[1])
    # Confirm that the drone_position and the goal position are not the same
    assert(not (drone_position[0] == goal_position[0] and drone_position[1] == goal_position[1]))
    # Confirm that the goal and drone position are free space
    assert(map_data[drone_position[0], drone_position[1]] == 0)
    assert(map_data[goal_position[0], goal_position[1]] == 0)

  # Expand the obstacles by distance so you do not hit one
  def expand_obstacles(self, map_data, distance):
    new_map = copy.deepcopy(map_data)
    # For each element in the map
    for i in range(map_data.shape[0]):
      for j in range(map_data.shape[1]):
        # if this is an obstacle
        if map_data[i, j] != 0:
          # Expand the obstacle by 1 in all directions
          for x_dim in range(-distance, distance + 1):
            for y_dim in range(-distance, distance + 1):
              if (0 <= i + x_dim < new_map.shape[0]) and(0 <= j + y_dim < new_map.shape[1]):
                new_map[i + x_dim, j + y_dim] = map_data[i, j]
    return new_map
