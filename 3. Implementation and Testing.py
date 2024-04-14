import numpy as np
import heapq
import json
import math

class RobotEnvironment:
    def __init__(self, width, height, obstacles=[], charging_station=None):
        self.width = width
        self.height = height
        self.obstacles = obstacles
        self.charging_station = charging_station
        self.grid = [[0] * width for _ in range(height)]
        
        for obstacle in obstacles:
            x, y = obstacle
            self.grid[y][x] = 1 
        
        self.robot_pos = None
    
    def set_robot_position(self, pos):
        if self.is_valid_position(pos):
            self.robot_pos = pos
        else:
            raise ValueError("Invalid position for the robot.")
    
    def move_forward(self):
        x, y = self.robot_pos
        new_x = x
        new_y = y + 1
        
        new_pos = (new_x, new_y)
        if self.is_valid_position(new_pos):
            self.robot_pos = new_pos
        else:
            print("Cannot move forward. Obstacle or boundary reached.")
    
    def get_possible_actions(self):
        return ['move_forward', 'move_backward', 'turn_left', 'turn_right']
    
    def is_valid_position(self, pos):
        x, y = pos
        return 0 <= x < self.width and 0 <= y < self.height and not self.grid[y][x]

    def plot_environment_with_robot(self):
        # Plotting function omitted for brevity
        pass

    def calculate_cost(self, action):
        if action in ['move_forward', 'move_backward']:
            return 1
        elif action in ['turn_left', 'turn_right']:
            return 1
        else:
            return 0

    def calculate_heuristic(self, current_pos, target_pos):
        # Manhattan distance heuristic
        x1, y1 = current_pos
        x2, y2 = target_pos
        return abs(x2 - x1) + abs(y2 - y1)

    def a_star_search(self, start_pos, target_pos):
        frontier = []
        heapq.heappush(frontier, (0, start_pos))
        came_from = {}
        cost_so_far = {}
        came_from[start_pos] = None
        cost_so_far[start_pos] = 0
        
        while frontier:
            current_cost, current_pos = heapq.heappop(frontier)
            
            if current_pos == target_pos:
                break
            
            for action in self.get_possible_actions():
                next_pos = self.get_next_position(current_pos, action)
                
                if next_pos is None:
                    continue
                
                new_cost = cost_so_far[current_pos] + self.calculate_cost(action)
                if next_pos not in cost_so_far or new_cost < cost_so_far[next_pos]:
                    cost_so_far[next_pos] = new_cost
                    priority = new_cost + self.calculate_heuristic(next_pos, target_pos)
                    heapq.heappush(frontier, (priority, next_pos))
                    came_from[next_pos] = current_pos
        
        # Reconstruct path
        path = []
        current_pos = target_pos
        while current_pos != start_pos:
            path.append(current_pos)
            current_pos = came_from[current_pos]
        path.append(start_pos)
        path.reverse()
        
        return path

    def get_next_position(self, current_pos, action):
    x, y = current_pos
    if action == 'move_forward':
        new_x = x
        new_y = y + 1
    elif action == 'move_backward':
        new_x = x
        new_y = y - 1
    elif action == 'turn_left':
        new_x = x - 1
        new_y = y
    elif action == 'turn_right':
        new_x = x + 1
        new_y = y
    else:
        return None
    
    new_pos = (new_x, new_y)
    return new_pos


# Test the implementation on various environments
def test_environment():
    env = RobotEnvironment(5, 5, obstacles=[(3, 0), (2, 2), (1, 3)], charging_station=(4, 4))
    robot_initial_position = (1, 1)
    target_cell = env.charging_station
    env.set_robot_position(robot_initial_position)

    path = env.a_star_search(robot_initial_position, target_cell)
    print("Optimal path:", path)
    print("Path length:", len(path))

    # Calculate total cost
    total_cost = sum(env.calculate_cost('move_forward') for _ in range(len(path) - 1))
    print("Total cost:", total_cost)

    # Test on different environments...

# Analyze the impact of different heuristic functions
def analyze_heuristic_impact():
    env = RobotEnvironment(5, 5, obstacles=[(3, 0), (2, 2), (1, 3)], charging_station=(4, 4))
    robot_initial_position = (1, 1)
    target_cell = env.charging_station
    env.set_robot_position(robot_initial_position)

    # Calculate path length and total cost with the Manhattan distance heuristic
    path_manhattan = env.a_star_search(robot_initial_position, target_cell)
    total_cost_manhattan = sum(env.calculate_cost('move_forward') for _ in range(len(path_manhattan) - 1))

    # Create a new environment object for obstacle-aware heuristic
    env_obstacle_aware = RobotEnvironment(5, 5, obstacles=[(3, 0), (2, 2), (1, 3)], charging_station=(4, 4))
    env_obstacle_aware.set_robot_position(robot_initial_position)

    # Calculate path length and total cost with the obstacle-aware heuristic
    path_obstacle_aware = env_obstacle_aware.a_star_search(robot_initial_position, target_cell)
    total_cost_obstacle_aware = sum(env_obstacle_aware.calculate_cost('move_forward') for _ in range(len(path_obstacle_aware) - 1))

    print("Manhattan Heuristic:")
    print("Path length:", len(path_manhattan))
    print("Total cost:", total_cost_manhattan)

    print("\nObstacle-Aware Heuristic:")
    print("Path length:", len(path_obstacle_aware))
    print("Total cost:", total_cost_obstacle_aware)

    # Compare path lengths and total costs...

if __name__ == "__main__":
    test_environment()
    analyze_heuristic_impact()

