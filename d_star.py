#!/usr/bin/env python

from graph import Node
import numpy as np

def scanEdges(node):
    return True

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
    return np.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2 + np.min((np.abs(s1.rot - s2.rot), 2 * np.pi - np.abs(s1.rot - s2.rot))) ** 2)

def cost(s1,s2): # TODO: Do we have to check for collisions here?
    return np.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2 + np.min((np.abs(s1.rot - s2.rot, 2 * np.pi - np.abs(s1.rot - s2.rot))) ** 2)

def cost_plus_g(s1, s2):
    return cost(s1,s2) + s2.g

def calculateKey(s): ### maybe change grid and k_m to global ###
	return [min([s.g, s.rhs]) + heuristic(graph.s_start, s) + k_m, min([s.g, s.rhs])]

def initialize(s_goal):
    U = pq.Priority_Queue
    k_m = 0
    s_goal.rhs = 0
    U.Insert(s_goal, calculateKey(s_goal))
    return U, k_m

def updateVertex(u): 
	if (u != graph.s_goal):
		u.rhs = min([ cost_plus_g(u, suc) for suc in u.succ()])
	if U.exist(u):
		U.delete(u)
	if u.g != u.rhs:
		U.push(u, calculateKey(u))

def computeShortestPath():
	while keyCompare(U.top()[0], calculateKey(graph.s_start)) or graph.s_start.rhs != graph.s_start.g:
		k_old = U.top()[0]
		u = U.pop()
		if keyCompare(k_old, calculateKey(u)):
			U.Insert(u, calculateKey(u))
		elif u.g > u.rhs:
			u.g = u.rhs
			for s in u.pred():
				updateVertex(s)
		else:
			u.g = np.inf
			updateVertex(u)
			for s in u.pred():
				updateVertex(s)


