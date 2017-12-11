#!/usr/bin/env python
#defines a basic node class
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
