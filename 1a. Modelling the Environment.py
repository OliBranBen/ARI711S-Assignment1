import numpy as np
import heapq
class CleaningEnvironment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [['empty' for _ in range(width)] for _ in range(height)] 

    def add_obstacle(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.grid[y][x] = 'obstacle'
        else:
            print("Error: Position out of bounds.")

    def display_environment(self):
        for row in self.grid:
            print(' '.join(row))


environment = CleaningEnvironment(5, 5) 

environment.add_obstacle(2, 2)  
environment.add_obstacle(3, 3)  
environment.add_obstacle(0, 1)  

environment.display_environment()

