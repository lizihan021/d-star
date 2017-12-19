#!/usr/bin/env python
#defines a basic node class
import numpy as np
import os

class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, g_in, rhs_in, coll_in):
        self.x = x_in # config position
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.g = g_in
        self.rhs = rhs_in
        self.coll = coll_in

    def succ(self, grid):
        successors = []
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if np.linalg.norm([i,j,k]) == 1:
                        if  self.id[0]+i >= 0 and self.id[0]+i < grid.x_grid_num and \
                            self.id[1]+j >= 0 and self.id[1]+j < grid.y_grid_num:
                            successors.append( grid.grid[self.id[0]+i][self.id[1]+j][(self.id[2]+k) % 4])
        return successors

    def pred(self, grid):
        return self.succ(grid)

    def j_id(self):
        return str(self.id)[1:-1]

    def __str__(self):
        return "\tNode id"+ str(self.id)+":"+ "x ="+ str(self.x)+ "y ="+str(self.y)+ "theta ="+ str(self.theta)

class Grid:
    def __init__(self, startconfig, goalconfig):
        self.step_len = 0.1
        self.x_len = 3.7
        self.y_len = 1.7
        # 0,      1,        2,  3,
        # -np.pi, -np.pi/2, 0, np.pi/2
        self.theta_num = 4
        self.x_grid_num = int(2*self.x_len/self.step_len)
        self.y_grid_num = int(2*self.y_len/self.step_len)
        self.grid = []
        # initialize the grid
        for i in range(self.x_grid_num):
            line = []
            for j in range(self.y_grid_num):
                line.append([ Node(-self.x_len+i*self.step_len,-self.y_len+j*self.step_len, np.pi/2*k - np.pi, (i,j,k),  np.inf, np.inf, False) for k in range(self.theta_num)])
            self.grid.append(line)
        # find start point and end point in the grid.
        self.start_n = self.getNode(self.config_point(startconfig))
        self.goal_n = self.getNode(self.config_point(goalconfig))
        self.start_n_no_change = Node(self.start_n.x,self.start_n.y, self.start_n.theta, self.start_n.id,  np.inf, np.inf, False)

    def printme(self,theta = 1):
        os.system('clear')
        for i in range(self.x_grid_num):
            for j in range(self.y_grid_num):
                print '{:01.1f}'.format(self.grid[i][j][theta].rhs),
            print ""

    def getNode(self, point):
        return self.grid[point[0]][point[1]][point[2]]

    def angle_index(self, angle):
        return int((angle + np.pi)/(np.pi/2) % 4)

    def index_angle(self, idx):
        return idx*np.pi/2 - np.pi

    def config_point(self, config):
        return [int(round((config[0]+self.x_len)/self.step_len)), int(round((config[1]+self.y_len)/self.step_len)), self.angle_index(config[2])]

    def point_config(self, point):
        return [point[0]*self.step_len - self.x_len, point[1]*self.step_len - self.y_len, self.index_angle(point[2])]