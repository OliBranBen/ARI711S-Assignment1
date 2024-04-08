import numpy as np
import heapq
class CleaningRobot:
    def __init__(self, environment_map, start, target):
        self.environment_map = environment_map
        self.start = start
        self.target = target
        self.actions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Up, Down, Right, Left

    def is_valid_move(self, position):
        x, y = position
        if 0 <= x < len(self.environment_map[0]) and 0 <= y < len(self.environment_map):
            return self.environment_map[y][x] != 'obstacle'
        return False

    def cost_function(self, action):
        # Define cost for each action (e.g., moving forward, backward, turning left, or turning right)
        return 1  # Unit cost for each movement

    def heuristic_function(self, current):
        # Define heuristic function to estimate remaining distance
        target_x, target_y = self.target
        current_x, current_y = current
        
        # Calculate Manhattan distance between current position and target position
        manhattan_distance = abs(target_x - current_x) + abs(target_y - current_y)
        
        # Penalty for navigating around obstacles
        obstacle_penalty = 0
        if self.environment_map[current_y][current_x] == 'obstacle':
            obstacle_penalty = 10  # Increase penalty for navigating through obstacles
        
        return manhattan_distance + obstacle_penalty

# Example usage
environment_map = [['empty', 'empty', 'obstacle', 'empty'],
                   ['empty', 'obstacle', 'empty', 'empty'],
                   ['empty', 'empty', 'empty', 'empty'],
                   ['empty', 'empty', 'obstacle', 'empty']]

start = (0, 0)
target = (3, 3)

robot = CleaningRobot(environment_map, start, target)
