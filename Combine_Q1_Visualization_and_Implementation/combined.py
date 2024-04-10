import numpy as np
import heapq
import matplotlib.pyplot as plt

class Environment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = set()

    def add_obstacle(self, x, y):
        self.obstacles.add((x, y))

    def is_obstacle(self, x, y):
        return (x, y) in self.obstacles

class Robot:
    def __init__(self, environment, start_position, target_position, charging_station):
        self.environment = environment
        self.position = start_position
        self.target = target_position
        self.charging_station = charging_station

    def actions(self, state):
        x, y = state
        possible_actions = []

        # Define possible movements (forward, backward, left, right)
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < self.environment.width and 0 <= new_y < self.environment.height:
                if not self.environment.is_obstacle(new_x, new_y):
                    possible_actions.append((new_x, new_y))

        return possible_actions

    def cost(self, state1, state2):
        # Simple unit cost for each movement
        return 1

    def heuristic(self, state):
        # Manhattan distance heuristic
        x1, y1 = state
        x2, y2 = self.target
        return abs(x1 - x2) + abs(y1 - y2)

    def a_star_search(self):
        frontier = [(self.heuristic(self.position), self.position)]
        came_from = {}
        cost_so_far = {self.position: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == self.target:
                break

            for next_state in self.actions(current):
                new_cost = cost_so_far[current] + self.cost(current, next_state)
                if next_state not in cost_so_far or new_cost < cost_so_far[next_state]:
                    cost_so_far[next_state] = new_cost
                    priority = new_cost + self.heuristic(next_state)
                    heapq.heappush(frontier, (priority, next_state))
                    came_from[next_state] = current

        # Reconstruct path
        path = []
        current = self.target
        while current != self.position:
            path.append(current)
            current = came_from[current]
        path.append(self.position)
        path.reverse()

        return path

# Example usage:
env = Environment(10, 10)
env.add_obstacle(2, 2)
env.add_obstacle(3, 2)
env.add_obstacle(4, 2)

robot = Robot(env, (0, 0), (9, 9), (5, 5))
path = robot.a_star_search()
print("Optimal path:", path)

# Visualization
def visualize_environment(robot, path):
    environment = robot.environment
    obstacles = environment.obstacles
    target_x, target_y = robot.target

    # Plot environment grid
    plt.figure(figsize=(8, 8))
    plt.imshow(np.zeros((environment.height, environment.width)), cmap='gray')

    # Plot obstacles
    for obstacle in obstacles:
        plt.text(obstacle[0], obstacle[1], 'X', color='red', ha='center', va='center', fontsize= 10, fontweight= 'bold')

    # Plot robot position
    plt.text(robot.position[0], robot.position[1], 'R', color='blue', ha='center', va='center', fontsize= 10, fontweight= 'bold')

    # Plot target position
    plt.text(target_x, target_y, 'T', color='green', ha='center', va='center', fontsize= 10, fontweight= 'bold')

    # Plot path
    if path:
        path_x, path_y = zip(*path)
        plt.plot(path_x, path_y, marker='o', markersize=10, color='orange')

    # Add legend
    legend_elements = [
        plt.Line2D([0], [0], marker='o', color='w', label='Robot', markerfacecolor='blue', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Target', markerfacecolor='green', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Obstacle', markerfacecolor='red', markersize=10),
        plt.Line2D([0], [0], marker='o', color='w', label='Path', markerfacecolor='orange', markersize=10),
    ]
    plt.legend(handles=legend_elements, loc='upper right')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Robot Environment Visualization')
    plt.grid(visible=True)
    plt.show()

# Testing visualization
visualize_environment(robot, path)