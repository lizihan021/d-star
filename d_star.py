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

### this is also c ###
###  ###
def heuristic(s1,s2):
    return np.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2 + np.min((np.abs(s1.theta - s2.theta), 2 * np.pi - np.abs(s1.theta - s2.theta))) ** 2)

def cost(s1,s2): # TODO: Do we have to check for collisions here?
    return np.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2 + np.min((np.abs(s1.theta - s2.theta), 2 * np.pi - np.abs(s1.theta - s2.theta))) ** 2)

def cost_plus_g(s1, s2):
    return cost(s1,s2) + s2.g

def calculateKey(s, grid, k_m): ### maybe change grid and k_m to global ###
	return [min([s.g, s.rhs])+h(grid.s_start, s)+k_m, min([s.g, s.rhs])]

def initialize(s_goal):
    U = pq.Priority_queue
    k_m = 0
    s_goal.rhs = 0
    U.insert(s_goal, calculateKey(s_goal))
    return U

def updateVertex(u, grid): 
	if (u != grid.s_goal):
		u.rhs = min([ cost_plus_g(u, suc) for suc in u.succ()])
	if U.exist(u):
		U.delete(u)
	if u.g != u.rhs:
		U.push(u, calculateKey(u))

def computeShortestPath(U, grid):
	while keyCompare(U.top()[0], calculateKey(grid.s_start)) or grid.s_start.rhs != grid.s_start.g:
		k_old = U.top()[0]
		u = U.pop()
		if keyCompare(k_old, calculateKey(u)):
			U.push(u, calculateKey(u))
		elif u.g > u.rhs:
			u.g = u.rhs
			for s in u.pred():
				updateVertex(s)
		else:
			u.g = np.inf
			updateVertex(u)
			for s in u.pred():
				updateVertex(s)


