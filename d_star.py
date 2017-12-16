#!/usr/bin/env python

from graph import Node
import numpy as np

def scanEdges(robot, env, node):
	changedEdges = False
	for neighbor in node.getNeighbors():
		config = coord_translator.coordToConfig(neighbor)
	    robot.SetActiveDOFValues(config)
		if env.CheckCollision(robot) and graph.getCost(node.getCoordinates(), neighbor) != np.inf: 
			changedEdges = True
			for collNeighbor in graph.getNode(neighbor).getNeighbors():
				graph.setCost(collNeighbor, neighbor, np.inf)
				updateVertex(collNeighbor)

		if not env.CheckCollision(robot) and graph.getCost(node.getCoordinates(), neighbor) == np.inf:
			changedEdges = True
			for nonCollNeighbor in graph.getNode(neighbor).getNeighbors():
				graph.setCost(nonCollNeighbor, neighbor, cost(graph.getNode(nonCollNeighbor), graph.getNode(neighbor)))
				updateVertex(nonCollNeighbor)
	return changedEdges


def keyCompare(lhs,rhs):
    if lhs[0] < rhs[0]:
        return True
    elif lhs[0] == rhs[0]:
        return lhs[1] < rhs[1]
    else:
        return False

def pqComparator(lhs,rhs):
    if lhs[0][0] < rhs[0][0]:
        return True
    elif lhs[0][0] == rhs[0][0]:
        return lhs[0][1] < rhs[0][1]
    else:
        return False

### this is also c ###
###  ###
def heuristic(s1,s2):
    return np.sqrt((s1.x - s2.x) ** 2 + (s1.y - s2.y) ** 2 + np.min((np.abs(s1.rot - s2.rot), 2 * np.pi - np.abs(s1.rot - s2.rot))) ** 2)

def cost(c1,c2): # TODO: Do we have to check for collisions here?
	s1 = coord_translator.coordToConfig(c1)
	s2 = coord_translator.coordToConfig(c2)
    return np.sqrt((s1[0] - s2[0]) ** 2 + (s1[1] - s2[1]) ** 2 + np.min((np.abs(s1[2] - s2[2], 2 * np.pi - np.abs(s1[2] - s2[2]))) ** 2))

def cost_plus_g(c1, c2):
    return graph.getCost(c1, c2) + graph.getNode(c2).g

def calculateKey(s): ### maybe change grid and k_m to global ###
	return [min([s.g, s.rhs]) + heuristic(graph.s_start, s) + k_m, min([s.g, s.rhs])]

def initialize(s_goal):
    U = pq.Priority_Queue(compare=pqComparator)
    k_m = 0
    s_goal.rhs = 0
    U.Insert(s_goal, calculateKey(s_goal))
    return U, k_m

def updateVertex(u_coord):
	if not graph.findNode(u_coord):
		tmp = Node(u_coord, np.inf, np.inf)
		graph.insertNode(tmp)
	for neighbor in graph.findNode(u_coord).getNeighbors():
		if not graph.findNode(neighbor):
			tmp = Node(neighbor, np.inf, np.inf)
			graph.insertNode(tmp)
	u = graph.getNode(u_coord)
	if (u_coord != graph.s_goal.getCoordinates()):
		u.rhs = min([ cost_plus_g(u_coord, neighbor) for neighbor in u.getNeighbors()])
	if U.exist(u):
		U.Remove(u)
	if u.g != u.rhs:
		U.Push(u, calculateKey(u))

def computeShortestPath():
	while keyCompare(U.Top()[0], calculateKey(graph.s_start)) or graph.s_start.rhs != graph.s_start.g:
		k_old = U.Top()[0]
		u = U.Pop()
		if keyCompare(k_old, calculateKey(u)):
			U.Insert(u, calculateKey(u))
		elif u.g > u.rhs:
			u.g = u.rhs
			for s in u.getNeighbors():
				updateVertex(s)
		else:
			u.g = np.inf
			updateVertex(u)
			for s in u.getNeighbors():
				updateVertex(s)


