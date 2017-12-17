#!/usr/bin/env python

import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from pq import Priority_Queue
from graph import *
import math
import cPickle as pickle
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
####################
# Helper Functions #
####################

def plotPoint(coord, ps_in, color_in, height_in):
    tmp = coord_translator.coordToConfig(coord)
    handles.append(env.plot3(points=np.array([tmp[0], tmp[1], height_in]),pointsize=ps_in,colors=color_in))

#########################
# D Star Implementation #
#########################
def scanEdges(robot, env, node):
    changedEdges = False
    for neighbor in node.getNeighbors():
        config = coord_translator.coordToConfig(neighbor)
        robot.SetActiveDOFValues(config)
        if env.CheckCollision(robot) and graph.getCost(node.getCoordinates(), neighbor) != np.inf: 
            # Red: Collision
            plotPoint(coord=neighbor, ps_in=7.0, color_in=[1,0,0], height_in=0.2)
            changedEdges = True
            for collNeighbor in graph.getNode(neighbor).getNeighbors():
                graph.setCost(collNeighbor, neighbor, np.inf)
                updateVertex(collNeighbor)

        if not env.CheckCollision(robot) and graph.getCost(node.getCoordinates(), neighbor) == np.inf:
            # Dark Teal: No longer Collision
            plotPoint(coord=neighbor, ps_in=7.0, color_in=[0,0.5,0.5], height_in=0.2)
            changedEdges = True
            for nonCollNeighbor in graph.getNode(neighbor).getNeighbors():
                graph.setCost(nonCollNeighbor, neighbor, cost(coord_translator, graph.getNode(nonCollNeighbor), graph.getNode(neighbor)))
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
    if lhs[0][0] > rhs[0][0]:
        return True
    elif lhs[0][0] == rhs[0][0]:
        return lhs[0][1] > rhs[0][1]
    else:
        return False

def heuristic(c1,c2):
    s1 = coord_translator.coordToConfig(c1)
    s2 = coord_translator.coordToConfig(c2)
    return np.sqrt( (s1[0] - s2[0]) ** 2 + (s1[1] - s2[1]) ** 2 +  np.min([np.abs(s1[2] - s2[2]), 2 * np.pi - np.abs(s1[2] - s2[2])]) ** 2)

def cost_plus_g(c1, c2):
    return graph.getCost(c1, c2) + graph.getNode(c2).g

def calculateKey(s): ### maybe change grid and k_m to global ###
    return [min([s.g, s.rhs]) + heuristic(graph.s_start.getCoordinates(), s.getCoordinates()) + k_m, min([s.g, s.rhs])]

def initialize(s_goal):
    U = Priority_Queue(compare=pqComparator)
    k_m = 0
    s_goal.rhs = 0
    U.Insert(s_goal, calculateKey(s_goal))
    return U, k_m

def updateVertex(u_coord):
    if not graph.findNode(u_coord):
        tmp = Node(u_coord, np.inf, np.inf)
        graph.insertNode(tmp, coord_translator)
    for neighbor in graph.getNode(u_coord).getNeighbors():
        if not graph.findNode(neighbor):
            tmp = Node(neighbor, np.inf, np.inf)
            graph.insertNode(tmp, coord_translator)
    u = graph.getNode(u_coord)
    if (u_coord != graph.s_goal.getCoordinates()):
        u.rhs = min([ cost_plus_g(u_coord, neighbor) for neighbor in u.getNeighbors()])
    if U.exist(u):
        U.Remove(u)
    if u.g != u.rhs:
        U.Insert(u, calculateKey(u))

def computeShortestPath(default_color):
    while keyCompare(U.Top()[0], calculateKey(graph.s_start)) or graph.s_start.rhs != graph.s_start.g:
        k_old = U.Top()[0]
        u = U.Pop()[1]
        # Teal: Computed Node
        plotPoint(coord=u.getCoordinates(), ps_in=7.0, color_in=default_color, height_in=0.1)
        if keyCompare(k_old, calculateKey(u)):
            U.Insert(u, calculateKey(u))
        elif u.g > u.rhs:
            u.g = u.rhs
            for s in u.getNeighbors():
                updateVertex(s)
        else:
            u.g = np.inf
            updateVertex(u.getCoordinates())
            for s in u.getNeighbors():
                updateVertex(s)

#############################
# End D Star Implementation #
#############################

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]
    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')    
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

####################
# Global Variables #
####################

U = 0
k_m = 0
graph = 0
coord_translator = 0
handles = []

if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

    o_pose = robot.GetTransform()
    start_config = [o_pose[0][3], o_pose[1][3], -pi/2]
    goal_config = [2.6,-1.3,-pi]
    ##########################
    #                        #
    # D* Lite Implementation #
    #                        #
    ##########################
    print "Start Configuration: ", start_config
    coord_translator = CoordinateTranslator(goal_config)
    s_start = Node(coord_translator.configToCoord(start_config), np.inf, np.inf)
    print "Relative StartCoordinates: ", s_start.getCoordinates()
    print "Goal Configuration: ", goal_config
    s_goal = Node([0,0,0], np.inf, np.inf)
    print "Relative Goal Coordinates: ", s_goal.getCoordinates()
    graph = Graph(s_start, s_goal)
    graph.insertNode(s_start, coord_translator)
    graph.insertNode(s_goal, coord_translator)

    default_color=[0,1,1]

    # Blue: Start Point
    plotPoint(coord=s_start.getCoordinates(), ps_in=7.0, color_in=[0,0,1], height_in=0.3)
    # Green: End Point
    plotPoint(coord=s_goal.getCoordinates(), ps_in=7.0, color_in=[0,1,0], height_in=0.3)
    # Start Timer
    raw_input("Press enter to continue")
    start = time.clock()
    ##############
    #            #
    #    Main    #
    #            #
    ##############
    s_last = graph.s_start
    U, k_m = initialize(graph.s_goal)
    computeShortestPath(default_color)
    # pickle.dump( {"a": U, "b": graph}, open( "save.p", "wb" ) )
    # tmp = pickle.load( open( "save.p", "rb" ) )
    # U, graph = tmp["a"], tmp["b"]

    default_color=[1,1,0]

    table6 = env.GetBodyFromEnvironmentId(6)
    # table7 = env.GetBodyFromEnvironmentId(7)
    new_pose6 = table6.GetTransform()
    new_pose6[0][3] = 0
    # new_pose7 = table7.GetTransform()
    # new_pose7[0][3] = 0
    table6.SetTransform(new_pose6)
    # table7.SetTransform(new_pose7)

    while np.linalg.norm(np.array(graph.s_start.getCoordinates()) - np.array(graph.s_goal.getCoordinates())) != 0:
        graph.s_start = graph.getNode(graph.s_start.getNeighbors()[np.argmin([cost_plus_g(graph.s_start.getCoordinates(), neighbor) for neighbor in graph.s_start.getNeighbors()])])
        # Black: Robot path
        plotPoint(coord=graph.s_start.getCoordinates(), ps_in=7.0, color_in=[0,0,0], height_in=0.2)
        robot.SetActiveDOFValues(coord_translator.coordToConfig(graph.s_start.getCoordinates()))
        with env:
            if scanEdges(robot, env, graph.s_start):
                # if there is edge value changed:
                k_m += heuristic(s_last.getCoordinates(), graph.s_start.getCoordinates())
                s_last = graph.s_start
                computeShortestPath(default_color)

    path = [] #put your final path in this variable

     #### End D* Lite Implementation  ###
    end = time.clock()
    print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")

