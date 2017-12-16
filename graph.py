#!/usr/bin/env python
#defines a basic node class
import numpy as np
from d_star import cost

class CoordinateTranslator:
    def __init__(self, goal_config_in):
        self.goal_config = goal_config_in
    
    def coordToConfig(self, coord):
        return [goal_config[0] + coord[0], goal_config[1] + coord[1], goal_config[2] + coord[2] * 0.5 * np.pi]

    def configToCoord(self, config):
        return [config[0] - goal_config[0], config[1] - goal_config[1], np.around((config[2] - goal_config[2]) / (0.5 * np.pi))]

class Node:
    def __init__(self, coord_in, g_in, rhs_in):
        self.x = coord_in[0]
        self.y = coord_in[1]
        self.rot = coord_in[2] 
        self.g = g_in
        self.rhs = rhs_in

    def getCoordinates(self):
        return [self.x, self.y, self.rot]

    def getNeighbors(self):
        neighbors = []
        for i in range(-1,2):
            for j in range(-1,2):
                for k in range(-1,2):
                    if np.linalg.norm([i,j,k]) != 0:
                        children.append([self.x + i, self.y + j, self.rot + k])
        return neighbors

    def printme(self):
        print "\tNode: x = ", self.x, " y =",self.y, " rot =", self.rot, " g(s) = ", self.g, "rhs(s) = ", self.rhs, " locally consistent = ", (self.g == self.rhs)

class Graph:
    # s_start_in, s_goal_in should be of Node class 
    def __init__(self, s_start_in, s_goal_in):
        self.s_start = s_start_in
        self.s_goal = s_goal_in
        self.nodes = {}
        self.cost = {}

    def insertNode(self, node):
        self.setNode(node)
        for neighbor in node.getNeighbors():
            if not self.findCost(neighbor, node.getCoordinates()):
                graph.setCost(node.getCoordinates(), neighbor, cost(node.getCoordinates(), neighbor))

    def setNode(self, node):
        coord = node.getCoordinates()
        self.node[(coord[0], coord[1], coord[2])] = node

    def getNode(self, coord):
        if not self.nodes.get((coord[0], coord[1], coord[2])):
            raise ValueError("Could not find node")
        return self.node[(coord[0], coord[1], coord[2])]

    def findNode(self, coord):
        return (coord[0], coord[1], coord[2]) in self.nodes

    def setCost(self, c1, c2, cost_in):
        self.cost[((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2]))] = cost_in
        self.cost[((c2[0],c2[1],c2[2]),(c1[0],c1[1],c1[2]))] = cost_in

    def getCost(self, c1, c2):
        if not self.cost.get(((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2]))):
            raise ValueError("Could not find cost")
        return self.cost.get(((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2])))

    def findCost(self, c1, c2):
        return ((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2])) in self.cost
