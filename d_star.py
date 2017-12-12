#!/usr/bin/env python

import numpy as np
import pq

def keyCompare(lhs,rhs):
    if lhs[0] < rhs[0]:
        return True
    elif lhs[0] == rhs[0]:
        return lhs[1] < rhs[1]
    else:
        return False

def heuristic(s1,s2):
    return np.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2 + np.min((np.abs(s1.theta - s2.theta), 2 * np.pi - np.abs(s1.theta - s2.theta))) ** 2)

def calculateRhs(s):
    return 0

def calculateG(s):
    return 0

def cost_plus_g(s1, s2):
    return heuristic(s1,s2) + calculateG(s1)

def calculateKey(s):
    return 0 # TODO: Set this up

def initialize():
    U = pq.Priority_queue
    k_m = 0
    for i in range(10): # TODO: Set this condition. for all s in S rhs(s) = g(s) = infinity
    rhs(s_goal) = 0
    U.push(s_goal, calculateKey(s_goal))
        print 'A'
    rhs(s_goal) = 0
    U.insert(s_goal, calculateKey(s_goal))
    return U

def updateVertex(u): #TODO: Do this LOL
    print 'A'

def computeShortestPath():
    return 0


