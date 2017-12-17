#!/usr/bin/env python
#defines a basic node class
import numpy as np

def cost(coord_translator, c1,c2): # TODO: Do we have to check for collisions here?
    s1 = coord_translator.coordToConfig(c1)
    s2 = coord_translator.coordToConfig(c2)
    return np.sqrt( (s1[0] - s2[0]) ** 2 + (s1[1] - s2[1]) ** 2 +  np.min([np.abs(s1[2] - s2[2]), 2 * np.pi - np.abs(s1[2] - s2[2])]) ** 2)

class CoordinateTranslator:
    def __init__(self, goal_config_in):
        self.goal_config = goal_config_in
    
    def coordToConfig(self, coord):
        return [self.goal_config[0] + 0.1 * coord[0], self.goal_config[1] + 0.1 * coord[1], self.goal_config[2] + coord[2] * 0.5 * np.pi]

    def configToCoord(self, config):
        return [np.around((config[0] - self.goal_config[0]) * 10), np.around((config[1] - self.goal_config[1]) * 10), np.around((config[2] - self.goal_config[2]) / (0.5 * np.pi)) % 4]

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
                    if np.linalg.norm([i,j,k]) == 1: # for 4 Connected, == 1, for 8 Connected, != 0
                        neighbors.append([self.x + i, self.y + j, (self.rot + k) % 4])
        return neighbors

    def __str__(self):
        return "\tNode: x = "+ str(self.x)+ " y = "+str(self.y)+ " rot = "+ str(self.rot)+ " g(s) = "+ str(self.g)+ " rhs(s) = "+ str(self.rhs)

class Graph:
    # s_start_in, s_goal_in should be of Node class 
    def __init__(self, s_start_in, s_goal_in):
        self.s_start = s_start_in
        self.s_goal = s_goal_in
        self.nodes = {}
        self.cost = {}

    def insertNode(self, node, coord_translator):
        self.setNode(node)
        for neighbor in node.getNeighbors():
            if not self.findCost(neighbor, node.getCoordinates()):
                self.setCost(node.getCoordinates(), neighbor, cost(coord_translator, node.getCoordinates(), neighbor))

    def setNode(self, node):
        coord = node.getCoordinates()
        self.nodes[(coord[0], coord[1], coord[2])] = node

    def getNode(self, coord):
        if not self.nodes.get((coord[0], coord[1], coord[2])):
            raise ValueError("Could not find node")
        return self.nodes[(coord[0], coord[1], coord[2])]

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

    def __str__(self):
        return str(self.nodes) +"\n"+ str(self.cost)
