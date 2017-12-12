
#!/usr/bin/env python

def keyCompare(lhs,rhs):
    if lhs[0] < rhs[0]:
        return True
    elif lhs[0] == rhs[0]:
        return lhs[1] < rhs[1]
    else:
        return False

def heuristic(s1,s2):
    return 0

def calculateRhs(s):
    return 0

def calculateG(s):
    return 0

def calculateKey(s):
    return 0 # TODO: Set this up

def Initialize():
    U = []
    k_m = 0
    for i in range(10): # TODO: Set this condition. for all s in S rhs(s) = g(s) = infinity
        print 'A'
    rhs(s_goal) = 0
    U.insert(s_goal, calculateKey(s_goal))

def updateVertex(u): #TODO: Do this LOL
    print 'A'

def computeShortestPath():
    return 0


