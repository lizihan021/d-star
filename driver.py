#!/usr/bin/env python

from pq import priority_queue

import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from d_star import *
from pq import Priority_Queue
from graph, import Node, CoorinateTranslator, Graph
import math
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

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
#                  #
# Global Variables #
#                  #
####################
U = 0
k_m = 0
graph = 0
coord_translator = 0

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

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        o_pose = robot.GetTransform()
        start_config = [o_pose[0][3], o_pose[1][3], 0]
        goal_config = [2.6,-1.3,-pi/2]
        handles = []
        start = time.clock()
        ##########################
        #                        #
        # D* Lite Implementation #
        #                        #
        ##########################
        s_start = Node(start_config, np.inf, np.inf)
        s_goal = Node(goal_config, np.inf, np.inf)
        graph = Graph(s_start, s_goal)
        graph.setNode(s_goal)
        for neighbor in s_goal.getNeighbors():`
            temp = Node(neighbor, np.inf, np.inf)
            graph.setCost(s_goal.getCoordinates(), neighbor, cost(s_goal, temp))
        coord_translator = CoordinateTranslator(goal_config)

        ##############
        #            #
        #    Main    #
        #            #
        ##############
        s_last = s_start 
        U, k_m = initialize(s_goal)
        computeShortestPath()
        while np.linalg.norm(np.array(grid.s_start) - np.array(grid.s_goal)) != 0:
            grid.s_start = grid.s_start.succ()[np.argmin([cost_plus_g(grid.s_start, suc) for suc in grid.s_start.succ()])]
            ### maybe ###
            robot.SetActiveDOFValues([grid.s_start.x, grid.s_start.y, grid.s_start.theta])
            if changed_flag:
                # if there is edge value changed:
                k_m += h(s_last, grid.s_start)
                s_last = grid.s_start
                some_path = computeShortestPath()
                changed_flag = False


        path = [] #put your final path in this variable

     #### End D* Lite Implementatio  ###
    end = time.clock()
    print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")

