# Import necessary libraries
import heapq
import matplotlib.pyplot as plt

# Visualization
def visualize_environment(robot, target_x, target_y, path, cleaned_cells):
    environment = robot.environment.grid

    #environment
    plt.imshow(environment, cmap='gray')

    #obstacles
    obstacles = [(i, j) for i in range(robot.environment.height) for j in range(robot.environment.width) if environment[i][j] == 1]
    for obstacle in obstacles:
        plt.text(obstacle[1], obstacle[0], 'X', color='red', ha='center', va='center')

    #robot position
    plt.text(robot.y, robot.x, 'R', color='blue', ha='center', va='center')

    #target position
    plt.text(target_y, target_x, 'T', color='green', ha='center', va='center')

    #cleaned cells
    for cell in cleaned_cells:
        plt.text(cell[1], cell[0], 'C', color='cyan', ha='center', va='center')

    #planned path
    if path:
        for position in path:
            plt.plot(position[1], position[0], marker='o', markersize=10, color='orange')

    # Legend
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', label='Cleaned Cell', markerfacecolor='cyan', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Planned Path', markerfacecolor='orange', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Robot', markerfacecolor='blue', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Target', markerfacecolor='green', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Obstacle', markerfacecolor='red', markersize=10),
    ]
    plt.legend(handles=legend_elements, loc='upper right')

    plt.gca().invert_yaxis()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Environment Visualization')
    plt.grid(visible=True)
    plt.show()

# Testing visualization
visualize_environment(robot, target_x, target_y, path_obstacle_aware, [])

'''
# Define the Environment class to model the grid environment
class Environment:
    def __init__(self, width, height):
        # Initialize the environment with specified width and height
        self.width = width
        self.height = height
        # Initialize the grid with all cells set to 0 (non-obstacle)
        self.grid = [[0 for _ in range(width)] for _ in range(height)]

    # Method to set a cell at (x, y) as an obstacle
    def set_obstacle(self, x, y):
        self.grid[y][x] = 1

    # Method to check if a cell at (x, y) is an obstacle
    def is_obstacle(self, x, y):
        return self.grid[y][x] == 1

# Define the Robot class
class Robot:
    def __init__(self, environment, x=0, y=0, direction='up'):
        # Initialize the robot with specified environment, position, and direction
        self.environment = environment
        self.x = x
        self.y = y
        self.direction = direction

    # Methods to move the robot forward and backward
    def move_forward(self):
        new_x, new_y = self.get_new_position()
        if self.is_valid_position(new_x, new_y):
            self.x = new_x
            self.y = new_y

    def move_backward(self):
        new_x, new_y = self.get_new_position(backward=True)
        if self.is_valid_position(new_x, new_y):
            self.x = new_x
            self.y = new_y

    # Methods to turn the robot left and right
    def turn_left(self):
        directions = ['up', 'left', 'down', 'right']
        current_index = directions.index(self.direction)
        self.direction = directions[(current_index - 1) % len(directions)]

    def turn_right(self):
        directions = ['up', 'right', 'down', 'left']
        current_index = directions.index(self.direction)
        self.direction = directions[(current_index + 1) % len(directions)]

    # Method to calculate the new position based on current direction
    def get_new_position(self, backward=False):
        delta = 1 if not backward else -1
        if self.direction == 'up':
            return self.x, self.y - delta
        elif self.direction == 'down':
            return self.x, self.y + delta
        elif self.direction == 'left':
            return self.x - delta, self.y
        elif self.direction == 'right':
            return self.x + delta, self.y

    # Method to check if a position is valid (within bounds and not an obstacle)
    def is_valid_position(self, x, y):
        return 0 <= x < self.environment.width and 0 <= y < self.environment.height and not self.environment.is_obstacle(x, y)

# Define CostFunction class to calculate costs
class CostFunction:
    def __init__(self):
        pass

    # Method to calculate cost of an action (currently returns a fixed cost of 1)
    def calculate_cost(self, action):
        return 1

# Define HeuristicFunction class to calculate heuristic values
class HeuristicFunction:
    def __init__(self, target_x, target_y, environment):
        self.target_x = target_x
        self.target_y = target_y
        self.environment = environment

    # Method to calculate Manhattan distance between two points
    def manhattan_distance(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)

    # Method to calculate heuristic value using A* algorithm (currently using Manhattan distance)
    def calculate_heuristic(self, x, y):
        return self.manhattan_distance(x, y, self.target_x, self.target_y)

# Define Node class for A* search
class Node:
    def __init__(self, x, y, g_cost, h_cost):
        self.x = x
        self.y = y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = None

    def __lt__(self, other):
        return self.f_cost < other.f_cost

# Define A* search algorithm
def astar_search(robot, target_x, target_y, cost_function, heuristic_function):
    # Initialize start node with robot's current position
    start_node = Node(robot.x, robot.y, 0, heuristic_function.calculate_heuristic(robot.x, robot.y))
    open_set = [start_node]
    closed_set = set()

    while open_set:
        current_node = heapq.heappop(open_set)

        if current_node.x == target_x and current_node.y == target_y:
            return construct_path(current_node)

        closed_set.add((current_node.x, current_node.y))

        neighbors = get_valid_neighbors(robot.environment, current_node.x, current_node.y)
        for neighbor in neighbors:
            if (neighbor.x, neighbor.y) in closed_set:
                continue

            tentative_g_cost = current_node.g_cost + cost_function.calculate_cost((neighbor.x, neighbor.y))
            if neighbor not in open_set or tentative_g_cost < neighbor.g_cost:
                neighbor.g_cost = tentative_g_cost
                neighbor.h_cost = heuristic_function.calculate_heuristic(neighbor.x, neighbor.y)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                neighbor.parent = current_node
                heapq.heappush(open_set, neighbor)

    return None  # No path found

# Define method to get valid neighbors for A* search
def get_valid_neighbors(environment, x, y):
    neighbors = []
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # Right, Left, Down, Up
    for dx, dy in directions:
        new_x, new_y = x + dx, y + dy
        if 0 <= new_x < environment.width and 0 <= new_y < environment.height \
                and not environment.is_obstacle(new_x, new_y):
            neighbors.append(Node(new_x, new_y, 0, 0))
    return neighbors

# Define method to construct path from goal node
def construct_path(node):
    path = []
    current = node
    while current is not None:
        path.append((current.x, current.y))
        current = current.parent
    return path[::-1]

# Example environment setup
env = Environment(width=5, height=5)
env.set_obstacle(2, 2)
env.set_obstacle(3, 2)
env.set_obstacle(3, 3)
env.set_obstacle(3, 4)
env.set_obstacle(1, 4)

robot = Robot(env)

# Example target location
target_x, target_y = 4, 4

# Simple Manhattan distance heuristic
simple_heuristic = HeuristicFunction(target_x, target_y, env)

# Obstacle-aware heuristic
obstacle_aware_heuristic = HeuristicFunction(target_x, target_y, env)

# A* search with simple heuristic
path_simple = astar_search(robot, target_x, target_y, CostFunction(), simple_heuristic)
print("Path with simple heuristic:", path_simple)

# A* search with obstacle-aware heuristic
path_obstacle_aware = astar_search(robot, target_x, target_y, CostFunction(), obstacle_aware_heuristic)
print("Path with obstacle-aware heuristic:", path_obstacle_aware)
'''






