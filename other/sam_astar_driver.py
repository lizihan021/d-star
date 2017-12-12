#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from pq import Priority_queue
from grid import Node, Grid
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
        def cal_cost(this_node, next_node, end_node):
            d_theta1 = this_node.theta - next_node.theta
            d_theta2 = end_node.theta - next_node.theta
            g = sqrt((this_node.x-next_node.x)**2+(this_node.y-next_node.y)**2+(min(d_theta1, 2*pi-d_theta1))**2)
            h = sqrt((end_node.x-next_node.x)**2+(end_node.y-next_node.y)**2+(min(d_theta2, 2*pi-d_theta2))**2)
            cost_o = this_node.cost
            next_node.cost = g + cost_o
            return g + h + cost_o

        def add_to_pq(q, this_p, end_p, grid, x_grid_num, y_grid_num, theta_num):
            this_node = grid[this_p[0]][this_p[1]][this_p[2]]
            robot.SetActiveDOFValues([this_node.x, this_node.y, this_node.theta])
            if not env.CheckCollision(robot):
                end_node = grid[end_p[0]][end_p[1]][end_p[2]]
                end_node = grid[end_p[0]][end_p[1]][end_p[2]]
                for i in [-1,0,1]:
                    for j in [-1,0,1]:
                        for k in [-1,0,1]:
                            if not (i==0 and j==0 and k==0):
                                if this_p[0]+i >= 0 and this_p[0]+i < x_grid_num and\
                                    this_p[1]+j >= 0 and this_p[1]+j < y_grid_num and\
                                    this_p[2]+k >= 0 and this_p[2]+k < theta_num:
                                    next_node = grid[this_p[0]+i][this_p[1]+j][this_p[2]+k]
                                    if next_node.parentid == (-1,-1,-1):
                                        next_node.parentid = this_node.id
                                        q.put((cal_cost(this_node, next_node, end_node), [this_p[0]+i,this_p[1]+j,this_p[2]+k] ))
                                        
        
        o_pose = robot.GetTransform()
        startconfig = [o_pose[0][3], o_pose[1][3], 0]
        step_len = 0.1
        x_grid_num = int(7.4/step_len)
        y_grid_num = int(3.4/step_len)
        theta_num = 4
        grid = []
        for i in range(x_grid_num):
            line = []
            for j in range(y_grid_num):
                line.append([ Node(-3.7+i*step_len,-1.7+j*step_len, pi-2*pi/theta_num*k, (i,j,k), (-1,-1,-1),0) for k in range(theta_num)])
            grid.append(line)
        start_p = [int(round((startconfig[0]+3.7)/step_len)), int(round((startconfig[1]+1.7)/step_len)), theta_num/2]
        end_p = [int(round((goalconfig[0]+3.7)/step_len)), int(round((goalconfig[1]+1.7)/step_len)), 3*theta_num/4]

        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
        q = PriorityQueue()
        grid[start_p[0]][start_p[1]][start_p[2]].parentid = (-2,-2,-2)
        add_to_pq(q, start_p, end_p, grid, x_grid_num, y_grid_num, theta_num)
        next_p = q.get()[1]
        while next_p != end_p:
            add_to_pq(q, next_p, end_p, grid, x_grid_num, y_grid_num, theta_num)
            if q.empty():
                print("no path")
                break
            next_p = q.get()
            f_cost = next_p[0]
            next_p = next_p[1]
            # print(next_p)
        print("final cost", f_cost)
        #### Draw the X and Y components of the configurations explored by your algorithm

 	path = [] #put your final path in this variable
        pp = grid[end_p[0]][end_p[1]][end_p[2]]
        while pp.parentid != (-2,-2,-2):
            path.append([pp.x,pp.y,pp.theta])
            id_p = pp.parentid
            # print(id_p)
            pp = grid[id_p[0]][id_p[1]][id_p[2]]

        robot.SetActiveDOFValues(startconfig)
        path.append(startconfig)
        path = path[::-1]
        pts = ()
        for p in path:
            pts = pts+((p[0], p[1], 0.05),)
        pts = array(pts)
        
        handles.append(env.drawlinestrip(points=pts,linewidth=3.0,
                                         colors=array(((0,0,0)))))


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

