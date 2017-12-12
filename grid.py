#!/usr/bin/env python
#defines a basic node class
from numpy import pi

class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in, cost_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in
        self.cost = cost_in

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid

class Grid:
	def __init__(self, startconfig, goalconfig):
        self.step_len = 0.1
        self.x_len = 3.7
        self.y_len = 1.7
        # -pi, -pi/2, 0, pi/2
        self.theta_num = 4
        self.x_grid_num = int(2*self.x_len/self.step_len)
        self.y_grid_num = int(2*self.y_len/self.step_len)
        self.grid = []
        for i in range(self.x_grid_num):
            line = []
            for j in range(self.y_grid_num):
                line.append([ Node(-self.x_len+i*self.step_len,-self.y_len+j*self.step_len, pi-2*pi/self.theta_num*k, (i,j,k), (-1,-1,-1),0) for k in range(self.theta_num)])
            self.grid.append(line)
        self.start_p = self.config_point(startconfig)
        self.start_p = self.config_point(goalconfig)

    def angle_index(self, angle):
    	return (angle + pi/2*3)/pi*2

    def config_point(self, config):
    	return [int(round((config[0]+self.x_len)/self.step_len)), int(round((config[1]+self.y_len)/self.step_len)), angle_index(config[2])]