# -*- coding: utf-8 -*-
"""
ENN584 Sensor Fusion practical.

Your task in this practical is to implement the incomplete methods below to 
perform occupancy grid mapping with a robot in a 2D environment equipped with:
    1. a laser scanner
    2. a radar scanner
    3. both, with early sensor fusion methods
    4. both, with late sensor fusion methods
The laser scanner is more precise than the radar scanner but has less range and
is vulnerable to occluding obstacles in the environment (a simulated version
of smoke).

Read through the code in both this file and the util_funcs.py script to
understand the tools you have available to you, but only edit code in this
script. You are free to edit code in this script in any way you like, the code
provided is merely a suggestion and skeleton for you to build on if you would
like.

For assessment in the following practical session you will need to demonstrate
the following things:
    1. functional occupancy mapping with each sensor individually
    2. an understanding of what early and late sensor fusion techniques mean
       and how they differentiate from each other. Where are they both useful?
    3. Attempts at implementing and testing sensor fusion to improve robustness
       under uncertainty, for the purpose of occupancy grid mapping.
    4. an understanding of how the code works, how changing certain parameters
       will affect performance, and of other core concepts behind occupancy
       mapping.
You will demonstrate these things by answering tutor questions, inspecting
variables and methods in a live python environment, and through the outputs of
your code as it runs (print statements, plots, graphs)


-------------------------------------------------------------------------------
Created by: Anthony Vanderkop, Thierry Peynot
Last edited: May 15, 2024
-------------------------------------------------------------------------------
"""
import os
import numpy as np
import matplotlib.pyplot as plt
from util_funcs import wrapToPi, Robot, Map, load_path, getLine

#a couple of other useful quantities to have
pi = np.pi
d2r = lambda x: x*pi/180
r2d = lambda x: x*180/pi


class OccupancyGrid(object):
    def __init__(self, map_bounds, resolution, occupied_threshold):
        self.resolution = resolution
        self.occupied_threshold = occupied_threshold
        self.world_bounds = map_bounds #in form [ xmin, ymin, xmax, ymax]
        
        self.x_origin = map_bounds[2] / 2
        self.y_origin =  map_bounds[3] / 2
        ij_min = self.world_to_ij(map_bounds[0], map_bounds[1])
        ij_max = self.world_to_ij(map_bounds[2], map_bounds[3])
        self.ij_bounds = [ij_min[0], ij_min[1], ij_max[0], ij_max[1]]
        self.grid = np.empty(self.ij_bounds[2:])
    
    def update(self, i: int, j: int, collision):
        '''
        Your code here. Update the values in the occupancy grid based on whether the laser
        collided with an obstacle at that position or not.
        Inputs:
        i, j: Integers. The row and column location the laser passed through, respectively
        collision: Boolean. True if it did collide, False otherwise.
        Outputs:
        None
        '''
        update_val = 0
        if collision:
            update_val = 1
        else:
            update_val = -0.3
        
        self.grid[i][j] += update_val

    def prob_2_log_odds(self, p):
        '''
        Your code here. Convert from log-odds to probability.
        @ChatGPT
        Inputs:
        p = probability. Float in range [0,1]
        
        Outputs:
        l - log-odds. Float from -Inf to +Inf
        '''
        return np.log(p / (1 - p))
        
    def log_odds_2_prob(self, l):
        '''
        Your code here. Convert from log-odds to probability.
        @ChatGPT
        Inputs:
        l - log-odds. Float from -Inf to +Inf
        Outputs:
        p = probability. Float in range [0,1]
        '''
        return 1 / (1 + np.exp(-l))
        
    def plot_occupancy_grid(self):
        plt.imshow(self.grid, cmap='gray', origin='lower')
        plt.colorbar()
        plt.title('Occupancy Grid')
        plt.xlabel('Column Index')
        plt.ylabel('Row Index')
        plt.show()
        
    def ij_to_world(self, i, j):
        x = (j - self.x_origin) * self.resolution
        y = -((i - self.y_origin) * self.resolution)
        return x, y
        
    def world_to_ij(self, x, y):
        i = self.y_origin + np.floor(-y / self.resolution)
        j = self.x_origin + np.floor((x) / self.resolution)
        return int(i), int(j)
    


def laser_scanner_occupancy(robot: Robot, occupancy_grid: OccupancyGrid):
    i = 0
    while True:
        laser_scan, _ = robot.step()
        for i, (range, collision) in enumerate(zip(laser_scan[0], laser_scan[1])):
            robot_x, robot_y = robot.pose[:2]
            bearing = wrapToPi(robot.pose[2] + d2r(-60 + 3 * i))
            dx = range * np.cos(bearing)
            dy = range * np.sin(bearing)
            measure_loc = [robot_x + dx, robot_y + dy]
            robot_i, robot_j = occupancy_grid.world_to_ij(robot_x, robot_y)
            meas_i, meas_j = occupancy_grid.world_to_ij(measure_loc[0], measure_loc[1])
            beam = getLine(robot_i, robot_j, meas_i, meas_j)
            plt.scatter(x=beam[:,0], y=beam[:,1])
            plt.pause(0.01)
            for i, (pi, pj) in enumerate(beam):
                if i == len(beam):
                    occupancy_grid.update(pi, pj, collision)
                else:
                    occupancy_grid.update(pi, pj, False)
        i += 1
        if i == 10:
            break
    occupancy_grid.plot_occupancy_grid()
    print(occupancy_grid.grid)
    
def radar_occupancy(robot, occupancy_grid):
    
    while True:
        _, radar_scan = robot.step()
        break
    
    raise NotImplementedError()
    
def sensor_fusion_occupancy_early(robot, occupancy_grid):
    
    while True:
        laser_scan, radar_scan = robot.step()
        break
        
    raise NotImplementedError()
    
def sensor_fusion_occupancy_late(robot, occupancy_grid):
    
    while True:
        laser_scan, radar_scan = robot.step()
        break
        
    raise NotImplementedError()

if __name__ == "__main__":
    
    #load in a map
    print(os.getcwd())
    mapfile = 'map.png'
    pathfile = 'map_realistic_path.txt'
    true_map = Map(mapfile, resolution=0.05, origin='centre')
    path = load_path(pathfile)
    
    bot = Robot(pose = path[0],
                true_map=true_map,
                path=path)
    
    map_min = list(true_map.to_world(0,0))
    map_max = list(true_map.to_world(true_map.shape[0], true_map.shape[1]))
    map_bounds = [map_min[0], map_min[1], map_max[0], map_max[1]]
    occupancy_grid = OccupancyGrid(map_bounds, resolution=0.05, occupied_threshold=0.7)
    
    #create the robot
    # robot = Robot()
    
    #continually update the occupancy map based on sensor measurements
    laser_scanner_occupancy(bot, occupancy_grid)