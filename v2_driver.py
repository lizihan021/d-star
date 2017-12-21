#!/usr/bin/env python

import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from pq import Priority_Queue
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

########################
# Graph Implementation #
########################

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

class Cost_calculator:
    def __init__(self, translator_in):
        self.translator = translator_in

    def calculateCost(c1,c2):
        s1 = self.translator.coordToConfig(c1)
        s2 = self.translator.coordToConfig(c2)
        return np.sqrt( (s1[0] - s2[0]) ** 2 + (s1[1] - s2[1]) ** 2 +  np.min([np.abs(s1[2] - s2[2]), 2 * np.pi - np.abs(s1[2] - s2[2])]) ** 2)

class Graph:
    # s_start_in, s_goal_in should be of type Node class 
    def __init__(self, s_start_in, s_goal_in, calc_cost_in):
        self.s_start = s_start_in
        self.s_goal = s_goal_in
        self.nodes = {}
        self.cost = {}
        self.calc_cost = calc_cost_in

    # Node find, get, set
    def findNode(self, coord):
        return (coord[0], coord[1], coord[2]) in self.nodes

    def insertNode(self, node):
        if not self.findNode(node.getCoordinates()):
            coord = node.getCoordinates()
            self.nodes[(coord[0], coord[1], coord[2])] = node
            for neighbor in node.getNeighbors():
                if not self.findCost(neighbor, node.getCoordinates()):
                    self.setCost(node.getCoordinates(), neighbor, self.calc_cost.calculateCost(node.getCoordinates(), neighbor))
    
    def getNode(self, coord):
        if not self.nodes.get((coord[0], coord[1], coord[2])):
            temp = Node(coord, np.inf, np.inf)
            self.insertNode(temp)
        return self.nodes[(coord[0], coord[1], coord[2])]

    def findCost(self, c1, c2):
        return ((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2])) in self.cost

    def setCost(self, c1, c2, cost_in):
        self.cost[((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2]))] = cost_in
        self.cost[((c2[0],c2[1],c2[2]),(c1[0],c1[1],c1[2]))] = cost_in

    def getCost(self, c1, c2):
        if not self.cost.get(((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2]))):
            raise ValueError("Could not find cost")
        return self.cost.get(((c1[0],c1[1],c1[2]),(c2[0],c2[1],c2[2])))

    def __str__(self):
        return str(self.nodes) +"\n"+ str(self.cost)

############################
# End Graph Implementation #
############################

####################
# Helper Functions #
####################
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

########################
# End Helper Functions #
########################

#########################
# D Star Lite Functions #
#########################

#############################
# End D Star Lite Functions #
#############################

####################
# Global Variables #
####################

U = 0
k_m = 0
graph = 0
coord_translator = 0
handles = []

########################
# End Global Variables #
########################

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
    start_config = [o_pose[0][3], o_pose[1][3], 0]
    goal_config = [2.6,-1.3,-pi/2]
    ##########################
    #                        #
    # D* Lite Implementation #
    #                        #
    ##########################
    coord_translator = CoordinateTranslator(goal_config)
    s_start = Node(coord_translator.configToCoord(start_config), np.inf, np.inf)
    s_goal = Node([0,0,0], np.inf, np.inf)
    graph = Graph(s_start, s_goal, coord_translator)
    graph.insertNode(s_start)
    graph.insertNode(s_goal)
    print "Start Configuration: ", start_config
    print "Relative StartCoordinates: ", s_start.getCoordinates()
    print "Goal Configuration: ", goal_config
    print "Relative Goal Coordinates: ", s_goal.getCoordinates()

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
            newEdges, k_m = scanEdges(robot, env, s_last, k_m)
            if newEdges:
                # if there is edge value changed:
                # k_m += cost(coord_translator, s_last.getCoordinates(), graph.s_start.getCoordinates())
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

