#!/usr/bin/env python
# coding: utf-8

# In[17]:


import numpy as np
import heapq
import matplotlib.pyplot as plt


# In[30]:


class RobotEnvironment:
    def __init__(self, width, height, obstacles=[]):
        self.width = width
        self.height = height
        self.obstacles = obstacles
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
        plt.figure(figsize=(self.width, self.height))
        
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] == 1:
                    plt.fill([x, x+1, x+1, x], [y, y, y+1, y+1], color='red')
        
        if self.robot_pos:
            plt.plot(self.robot_pos[0], self.robot_pos[1], 'bo', markersize=10, label='Robot Position')
        
        for action in self.get_possible_actions():
            if action == 'move_forward':
                plt.arrow(self.robot_pos[0], self.robot_pos[1], 0, 1, head_width=0.2, head_length=0.1, fc='g', ec='g')
            elif action == 'move_backward':
                plt.arrow(self.robot_pos[0], self.robot_pos[1], 0, -1, head_width=0.2, head_length=0.1, fc='g', ec='g')
            elif action == 'turn_left':
                plt.arrow(self.robot_pos[0], self.robot_pos[1], -1, 0, head_width=0.2, head_length=0.1, fc='g', ec='g')
            elif action == 'turn_right':
                plt.arrow(self.robot_pos[0], self.robot_pos[1], 1, 0, head_width=0.2, head_length=0.1, fc='g', ec='g')
        
        plt.title('Robot Environment')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.xticks(range(self.width))
        plt.yticks(range(self.height))
        plt.grid(color='black', linestyle='-', linewidth=1)
        plt.legend()
        plt.show()

# Example usage:
env = RobotEnvironment(5, 5, obstacles=[(3, 0), (2, 2)])
robot_initial_position = (1, 1)
env.set_robot_position(robot_initial_position)

env.move_forward()  # Move the robot forward
env.plot_environment_with_robot()


# In[ ]:




