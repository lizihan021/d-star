#!/usr/bin/env python
#defines a basic node class
import numpy as np

class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in, g_in, rhs_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in
        self.g = g_in
        self.rhs = rhs_in

    def succ(self, grid):
        successors = []
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if np.linalg.norm([i,j,k]) != 0:
                        if  self.id[0]+i >= 0 and self.id[0]+i < grid.x_grid_num \
                            self.id[1]+j >= 0 and self.id[1]+j < grid.y_grid_num:
                            successors.append( grid.grid[self.id[0]+i][self.id[1]+j][(self.id[2]+k) % 4])
        return successors

    def pred(self, grid):
        return self.succ(grid)

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid

class Grid:
    def __init__(self, startconfig, goalconfig):
        self.step_len = 0.1
        self.x_len = 3.7
        self.y_len = 1.7
        # -np.pi, -np.pi/2, 0, np.pi/2
        self.theta_num = 4
        self.x_grid_num = int(2*self.x_len/self.step_len)
        self.y_grid_num = int(2*self.y_len/self.step_len)
        self.grid = []
        for i in range(self.x_grid_num):
            line = []
            for j in range(self.y_grid_num):
                line.append([ Node(-self.x_len+i*self.step_len,-self.y_len+j*self.step_len, np.pi-2*np.pi/self.theta_num*k, (i,j,k), (-1,-1,-1), np.inf, np.inf ) for k in range(self.theta_num)])
            self.grid.append(line)
        self.s_start = self.config_point(startconfig)
        self.s_goal = self.config_point(goalconfig)

    def angle_index(self, angle):
        return (angle + np.pi/2*3)/np.pi*2

    def config_point(self, config):
        return [int(round((config[0]+self.x_len)/self.step_len)), int(round((config[1]+self.y_len)/self.step_len)), angle_index(config[2])]