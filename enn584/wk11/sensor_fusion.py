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
from PIL import Image 

import os
import numpy as np
import matplotlib.pyplot as plt
import copy

from util_funcs import wrapToPi, Robot, Map, load_path, getLineFromGridCoord

#a couple of other useful quantities to have
pi = np.pi
d2r = lambda x: x*pi/180
r2d = lambda x: x*180/pi


class OccupancyGrid(object):
    def __init__(self, map_bounds, resolution, occupied_threshold):
        self.resolution = resolution # m / px
        self.occupied_threshold = occupied_threshold
        self.bounds = map_bounds #in form [ xmin, ymax, xmax, ymin]

        self.x_range = np.abs(self.bounds[2] - self.bounds[0])
        self.y_range = np.abs(self.bounds[1] - self.bounds[3])
        self.i_max = int(self.y_range / self.resolution)
        self.j_max = int(self.x_range / self.resolution)
        self.x_origin = self.j_max / 2
        self.y_origin = self.i_max / 2
        
        self.grid = np.zeros([self.i_max, self.j_max])
        
        self.isPlotting = False
    
    def update(self, i: int, j: int, collision, scaling=1):
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
        
        self.grid[i][j] = self.grid[i][j] + update_val * scaling

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
        prob = np.zeros((self.i_max, self.j_max))
        for i in range(self.i_max):
            for j in range(self.j_max):
                prob[i][j] = self.log_odds_2_prob(self.grid[i][j])
        plt.imshow(prob, cmap='gray_r', origin='lower')
        if (not self.isPlotting):
            plt.colorbar()
            plt.title('Occupancy Grid')
            plt.xlabel('Column Index')
            plt.ylabel('Row Index')
        self.isPlotting = True
        plt.waitforbuttonpress(1)
        
    def ij_to_world(self, i, j):
        x = (j - self.x_origin) * self.resolution
        y = -((i - self.y_origin) * self.resolution)
        return x, y
        
    def world_to_ij(self, x, y):
        i = self.y_origin + y / self.resolution
        j = self.x_origin + (x) / self.resolution
        return int(i), int(j)
    


def laser_scanner_occupancy(robot: Robot, occupancy_grid: OccupancyGrid):
    i = 0
    while True:
        laser_scan, _ = robot.step()
        robot_x, robot_y = robot.pose[:2]
        for i, (range, collision) in enumerate(zip(laser_scan[0], laser_scan[1])):
            bearing = wrapToPi(robot.pose[2] + robot.laser_angles[i])
            laser_x = robot_x + range * np.cos(bearing)
            laser_y = robot_y + range * np.sin(bearing)
            
            robot_i, robot_j = occupancy_grid.world_to_ij(robot_x, robot_y)
            laser_i, laser_j = occupancy_grid.world_to_ij(laser_x, laser_y)
            laser_i = min(occupancy_grid.i_max - 1, laser_i)
            laser_j = min(occupancy_grid.j_max - 1, laser_j)
            beam = getLineFromGridCoord(robot_i, robot_j, laser_i, laser_j)
            for pi, pj in beam:
                if (pi, pj) == (laser_i, laser_j):
                    occupancy_grid.update(pi, pj, collision)
                else:
                    occupancy_grid.update(pi, pj, False)
        occupancy_grid.plot_occupancy_grid()


def radar_occupancy(robot: Robot, occupancy_grid: OccupancyGrid):
    while True:
        _, radar_scan = robot.step()
        robot_x, robot_y = robot.pose[:2]
        for i, (range, collision) in enumerate(zip(radar_scan[0], radar_scan[1])):
            bearing = wrapToPi(robot.pose[2] + robot.radar_angles[i])
            radar_x = robot_x + range * np.cos(bearing)
            radar_y = robot_y + range * np.sin(bearing)
            
            robot_i, robot_j = occupancy_grid.world_to_ij(robot_x, robot_y)
            radar_i, radar_j = occupancy_grid.world_to_ij(radar_x, radar_y)
            radar_i = min(occupancy_grid.i_max - 1, radar_i)
            radar_j = min(occupancy_grid.j_max - 1, radar_j)
            beam = getLineFromGridCoord(robot_i, robot_j, radar_i, radar_j)
            for pi, pj in beam:
                if (pi, pj) == (radar_i, radar_j):
                    occupancy_grid.update(pi, pj, collision)
                else:
                    occupancy_grid.update(pi, pj, False)
        occupancy_grid.plot_occupancy_grid()
    
    
def sensor_fusion_occupancy_early(robot: Robot, occupancy_grid: OccupancyGrid):
    laser_scaling = 1 - robot.laser_covariance[0]
    radar_scaling = 1 - robot.radar_covariance[0]
    
    while True:
        laser_scan, radar_scan = robot.step()
        robot_x, robot_y = robot.pose[:2]
        
        # Laser Occupancy
        for i, (range, collision) in enumerate(zip(laser_scan[0], laser_scan[1])):
            bearing = wrapToPi(robot.pose[2] + robot.laser_angles[i])
            laser_x = robot_x + range * np.cos(bearing)
            laser_y = robot_y + range * np.sin(bearing)
            
            robot_i, robot_j = occupancy_grid.world_to_ij(robot_x, robot_y)
            laser_i, laser_j = occupancy_grid.world_to_ij(laser_x, laser_y)
            laser_i = min(occupancy_grid.i_max - 1, laser_i)
            laser_j = min(occupancy_grid.j_max - 1, laser_j)
            beam = getLineFromGridCoord(robot_i, robot_j, laser_i, laser_j)
            for pi, pj in beam:
                if (pi, pj) == (laser_i, laser_j):
                    occupancy_grid.update(pi, pj, collision, scaling=laser_scaling)
                else:
                    occupancy_grid.update(pi, pj, False, scaling=laser_scaling)
                    
        # Radar Occupancy
        for i, (range, collision) in enumerate(zip(radar_scan[0], radar_scan[1])):
            bearing = wrapToPi(robot.pose[2] + robot.radar_angles[i])
            radar_x = robot_x + range * np.cos(bearing)
            radar_y = robot_y + range * np.sin(bearing)
            
            robot_i, robot_j = occupancy_grid.world_to_ij(robot_x, robot_y)
            radar_i, radar_j = occupancy_grid.world_to_ij(radar_x, radar_y)
            radar_i = min(occupancy_grid.i_max - 1, radar_i)
            radar_j = min(occupancy_grid.j_max - 1, radar_j)
            beam = getLineFromGridCoord(robot_i, robot_j, radar_i, radar_j)
            for pi, pj in beam:
                if (pi, pj) == (radar_i, radar_j):
                    occupancy_grid.update(pi, pj, collision, scaling=radar_scaling)
                else:
                    occupancy_grid.update(pi, pj, False, scaling=radar_scaling)
        occupancy_grid.plot_occupancy_grid()
        
    
def sensor_fusion_occupancy_late(robot: Robot, occupancy_grid: OccupancyGrid):
    laser_weighting = 1 / robot.laser_covariance[0]
    radar_weighting = 1 / robot.radar_covariance[0]
    total_weight = laser_weighting + radar_weighting
    while True:
        laser_occupancy: OccupancyGrid = copy.copy(occupancy_grid)
        radar_occupancy: OccupancyGrid = copy.copy(occupancy_grid)
        laser_scan, radar_scan = robot.step()
        robot_x, robot_y = robot.pose[:2]
        
        # Laser Occupancy
        for i, (range, collision) in enumerate(zip(laser_scan[0], laser_scan[1])):
            bearing = wrapToPi(robot.pose[2] + robot.laser_angles[i])
            laser_x = robot_x + range * np.cos(bearing)
            laser_y = robot_y + range * np.sin(bearing)
            
            robot_i, robot_j = laser_occupancy.world_to_ij(robot_x, robot_y)
            laser_i, laser_j = laser_occupancy.world_to_ij(laser_x, laser_y)
            laser_i = min(laser_occupancy.i_max - 1, laser_i)
            laser_j = min(laser_occupancy.j_max - 1, laser_j)
            beam = getLineFromGridCoord(robot_i, robot_j, laser_i, laser_j)
            for pi, pj in beam:
                if (pi, pj) == (laser_i, laser_j):
                    laser_occupancy.update(pi, pj, collision)
                else:
                    laser_occupancy.update(pi, pj, False)
                    
        # Radar Occupancy
        for i, (range, collision) in enumerate(zip(radar_scan[0], radar_scan[1])):
            bearing = wrapToPi(robot.pose[2] + robot.radar_angles[i])
            radar_x = robot_x + range * np.cos(bearing)
            radar_y = robot_y + range * np.sin(bearing)
            
            robot_i, robot_j = radar_occupancy.world_to_ij(robot_x, robot_y)
            radar_i, radar_j = radar_occupancy.world_to_ij(radar_x, radar_y)
            radar_i = min(radar_occupancy.i_max - 1, radar_i)
            radar_j = min(radar_occupancy.j_max - 1, radar_j)
            beam = getLineFromGridCoord(robot_i, robot_j, radar_i, radar_j)
            for pi, pj in beam:
                if (pi, pj) == (radar_i, radar_j):
                    radar_occupancy.update(pi, pj, collision)
                else:
                    radar_occupancy.update(pi, pj, False)
                    
        # Fusion
        occupancy_grid.grid = (laser_occupancy.grid * laser_weighting 
                               + radar_occupancy.grid * radar_weighting) / total_weight
        occupancy_grid.plot_occupancy_grid()
        # print(occupancy_grid.grid)
        


if __name__ == "__main__":    
    #load in a map
    mapfile = 'map1_cloudy.png'
    pathfile = 'map1_path.txt'
    true_map = Map(mapfile, resolution=0.02, origin='centre', occlusion_prob=0.05)
    path = load_path(pathfile)
    
    bot = Robot(pose = path[0],
                true_map=true_map,
                path=path, 
                # pose_covariance=[0,0,0],
                # laser_covariance=[0,0],
                noise=True)
    
    map_min = list(true_map.to_world(0,0))
    map_max = list(true_map.to_world(true_map.shape[0], true_map.shape[1]))
    map_bounds = [map_min[0], map_min[1], map_max[0], map_max[1]]
    occupancy_grid = OccupancyGrid(map_bounds, resolution=0.05, occupied_threshold=0.7)
    
    
    #continually update the occupancy map based on sensor measurements
    # laser_scanner_occupancy(bot, occupancy_grid)
    # radar_occupancy(bot, occupancy_grid)
    # sensor_fusion_occupancy_early(bot, occupancy_grid)
    sensor_fusion_occupancy_late(bot, occupancy_grid)