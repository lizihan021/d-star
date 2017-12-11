#!/usr/bin/env python

from pq import priority_queue

import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from Queue import PriorityQueue
from pq import Node
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

        goalconfig = [2.6,-1.3,-pi/2]
	start = time.clock()

        #### YOUR CODE HERE ####
        handles = []
        o_pose = robot.GetTransform()
        startconfig = [o_pose[0][3], o_pose[1][3], 0]
        step_len = 0.1
        x_grid_len = 3.7
        y_grid_len = 1.7
        x_grid_num = int(2*x_grid_len/step_len)
        y_grid_num = int(2*y_grid_len/step_len)
        theta_num = 4
        grid = []
        for i in range(x_grid_num):
            line = []
            for j in range(y_grid_num):
                line.append([ Node(-3.7+i*step_len,-1.7+j*step_len, pi-2*pi/theta_num*k, (i,j,k), (-1,-1,-1),0) for k in range(theta_num)])
            grid.append(line)
        start_p = [int(round((startconfig[0]+3.7)/step_len)), int(round((startconfig[1]+1.7)/step_len)), theta_num/2]
        end_p = [int(round((goalconfig[0]+3.7)/step_len)), int(round((goalconfig[1]+1.7)/step_len)), 3*theta_num/4]


 	path = [] #put your final path in this variable

 	#### END OF YOUR CODE ###
	end = time.clock()
	print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
	traj = ConvertPathToTrajectory(robot, path)

	# Execute the trajectory on the robot.
	if traj != None:
	    robot.GetController().SetPath(traj)


    waitrobot(robot)

    raw_input("Press enter to exit...")

