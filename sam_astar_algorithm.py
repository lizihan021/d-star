#!/usr/bin/env python
# -*- coding: utf-8 -*-
import openravepy
import numpy
from Queue import PriorityQueue

def actionCost(current, neighbor):
    return numpy.sqrt((current[0] - neighbor[0]) ** 2 + (current[1] - neighbor[1]) ** 2 + numpy.min((numpy.abs(current[2] - neighbor[2]),2 * numpy.pi - numpy.abs(current[2] - neighbor[2]))))

def heuristicCost(current, goal):
    return numpy.sqrt((current[0] - goal[0]) ** 2 + (current[1] - goal[1]) ** 2 + numpy.min((numpy.abs(current[2] - goal[2]),2 * numpy.pi - numpy.abs(current[2] - goal[2]))))

def tracePath(tupleMap, start, goal):
    path = []
    previous = goal
    while previous != start:
        path.append([previous[0], previous[1], previous[2]])
        previous = tupleMap[previous][0]
    path.append(start)
    path.reverse()
    return path

def getValidNeighbors(env, robot, current, four_connect, collisions):
    neighbors = []
    for i in range(-1,2):
        for j in range(-1,2):
            for k in range(-1,2):
                if four_connect:
                    if abs(i) + abs(j) + abs(k) == 1:
                        theta = (current[2] + (k * numpy.pi / 2)) % (2 * numpy.pi)
                        test_tuple = tuple((current[0] + (0.1*i), current[1] + (0.1*j), theta))
                        robot.SetActiveDOFValues(test_tuple)
                        if not env.CheckCollision(robot):
                            neighbors.append(test_tuple)
                        else:
                            collisions.append([test_tuple[0], test_tuple[1],0.1])
                else:
                    if abs(i) + abs(j) + abs(k) != 0:
                        theta = (current[2] + (k * numpy.pi / 2)) % (2 * numpy.pi)
                        test_tuple = tuple((current[0] + (0.1*i), current[1] + (0.1*j), theta))
                        robot.SetActiveDOFValues(test_tuple)
                        if not env.CheckCollision(robot):
                            neighbors.append(test_tuple)
                        else:
                            collisions.append([test_tuple[0], test_tuple[1],0.1])
    return neighbors

def AStar(env, robot, goal, four_connect):
    path = []
    collisions = []
    all_searched = []
    # Get's the starting configuration and turns it into a tuple
    start = robot.GetActiveDOFValues()
    start = tuple((start[0],start[1],start[2] % (2 * numpy.pi)))
    # Initialize the PQ containing the start location
    astar_pq = PriorityQueue()
    astar_pq.put((heuristicCost(start,goal),start))
    # Data Structures to be used in A-Star
    closedSet = {}
    tupleMap = {}
    updatedG = {}
    tupleMap[start] = tuple((start,0))
    updatedG[start] = 0
    # Begin A* Search
    while not astar_pq.empty():
        current = astar_pq.get()[1]
        if heuristicCost(current,goal) < 0.00001:
            print "Total Cost:", tupleMap[current][1]
            path = tracePath(tupleMap, start, current)
            return path, collisions, all_searched
        closedSet[current] = 1
        currentNeighbors = getValidNeighbors(env, robot, current, four_connect, collisions)
        for neighbor in currentNeighbors:
            if neighbor in closedSet:
                continue
            gCost = tupleMap[current][1] + actionCost(current, neighbor)
            hCost = heuristicCost(neighbor, goal)
            if neighbor not in tupleMap:
                all_searched.append([neighbor[0], neighbor[1], 0.1])
                tupleMap[neighbor] = tuple((current,gCost))
                astar_pq.put((gCost + hCost, neighbor))
            if gCost >= tupleMap[neighbor][1]:
                continue
    return path, collisions, all_searched
