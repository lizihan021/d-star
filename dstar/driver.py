#!/usr/bin/env python
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from pq import Priority_Queue
from grid import Node, Grid
import math
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    import numpy as np

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
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=2*np.ones(3),maxaccelerations=7*np.ones(3))
    return traj

####################
# Global Variables #
####################
k_m = 0
U = 0
grid = 0
handles = []
plot_check = {}
####################
# D Star Functions #
####################
def getNode(id_in):
    point = id_in.split(", ")
    return grid.grid[int(point[0])][int(point[1])][int(point[2])]

def plotPoint(node, ps_in, color_in, height_in):
    co_str = str(node.id)
    col_str = str(color_in)
    if not co_str in plot_check:
        plot_check[co_str] = col_str
        handles.append(env.plot3(points=np.array([node.x, node.y, height_in]),pointsize=ps_in,colors=color_in))
    else:
        if plot_check[co_str] != col_str:
            plot_check[co_str] = col_str
            handles.append(env.plot3(points=np.array([node.x, node.y, height_in]),pointsize=ps_in,colors=color_in))

def keyCompare(lhs,rhs):
    if lhs[0] < rhs[0]:
        return True
    elif lhs[0] == rhs[0]:
        return lhs[1] < rhs[1]
    else:
        return False

def heuristic(n1, n2):
    return np.sqrt( (n1.x - n2.x)**2 + (n1.y - n2.y)**2 +  np.min([np.abs(n1.theta - n2.theta), 2 * np.pi - np.abs(n1.theta - n2.theta)])**2)

def cost(n1, n2):
    if n1.coll or n2.coll:
        return np.inf
    else:
        return np.sqrt( (n1.x - n2.x)**2 + (n1.y - n2.y)**2 +  np.min([np.abs(n1.theta - n2.theta), 2 * np.pi - np.abs(n1.theta - n2.theta)])**2)

def cost_plus_g(n1, n2):
    return cost(n1, n2) + n2.g

def calculateKey(node):
    return [min([node.g, node.rhs]) + heuristic(grid.start_n_no_change, node) + k_m, min([node.g, node.rhs])]

def initialize():
    U = Priority_Queue()
    k_m = 0
    grid.goal_n.rhs = 0
    U.Insert(grid.goal_n.j_id(), calculateKey(grid.goal_n))
    return U, k_m

def checkCollision(robot, env):
    changed = False
    changed_n_1 = []
    changed_n = []
    for node in grid.start_n.succ(grid):
        robot.SetActiveDOFValues(grid.point_config(node.id))
        coll = env.CheckCollision(robot)
        if coll and not node.coll:
            changed = True
            node.coll = True
            node.g = np.inf
            node.rhs = np.inf
            changed_n_1.append(node)
            plotPoint(node, 5, [1,0,0], 0.1)
        elif not coll and node.coll:
            changed = True
            node.coll = False
            changed_n_1.append(node)
            changed_n.append(node)
    changed_n.append(grid.start_n)
    for node in changed_n_1:
        if node.coll:
            for succ in node.succ(grid):
                if not succ.coll:
                    if not succ in changed_n:
                        changed_n.append(succ)
    return True, changed_n


def updateVertex(u): 
    if not u.coll:
        if (u != grid.goal_n):
            u.rhs = min([ cost_plus_g(u, suc) for suc in u.succ(grid)])
        if U.Exist(u.j_id()):
            U.Remove(u.j_id())
        if u.g != u.rhs:
            U.Insert(u.j_id(), calculateKey(u))

def computeShortestPath():
    while keyCompare(U.Top()[0], calculateKey(grid.start_n)) or grid.start_n.rhs != grid.start_n.g:
        k_old = U.Top()[0]
        u = getNode(U.Pop()[1])
        if u.coll:
            continue
        if keyCompare(k_old, calculateKey(u)):
            U.Insert(u.j_id(), calculateKey(u))
        elif u.g > u.rhs:
            u.g = u.rhs
            for s in u.pred(grid):
                updateVertex(s)
        else:
            u.g = np.inf
            updateVertex(u)
            for s in u.pred(grid):
                updateVertex(s)
########################
# End D Star Functions #
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
    goalconfig = [2.6,-1.3,-np.pi/2]
    raw_input("Press enter to continue")
    start = time.clock()
    ##########################
    # D* Lite Implementation #
    ##########################
    
    o_pose = robot.GetTransform()
    startconfig = [o_pose[0][3], o_pose[1][3], 0]
    grid = Grid(startconfig, goalconfig)

    ##############
    #    Main    #
    ##############
    last_n = grid.start_n
    U, k_m = initialize()
    computeShortestPath()

    table4 = env.GetBodyFromEnvironmentId(4)
    new_pose4 = table4.GetTransform()

    table6 = env.GetBodyFromEnvironmentId(6)
    new_pose6 = table6.GetTransform()
    new_pose6[0][3] = 0
    table6.SetTransform(new_pose6)

    table7 = env.GetBodyFromEnvironmentId(7)
    new_pose7 = table7.GetTransform()
    new_pose7[0][3] = 2
    new_pose7[1][3] = 0.5
    table7.SetTransform(new_pose7)
    count = 0
    previous = [grid.start_n.x, grid.start_n.y, grid.start_n.theta]

    while grid.start_n != grid.goal_n:
        grid.start_n = grid.start_n.succ(grid)[np.argmin([cost_plus_g(grid.start_n, suc) for suc in grid.start_n.succ(grid)])]
        ### maybe ###
        #print grid.start_n.id, grid.start_n.theta
        #robot.SetActiveDOFValues([grid.start_n.x, grid.start_n.y, grid.start_n.theta])

        path = [[grid.start_n.x, grid.start_n.y, grid.start_n.theta], previous]
        traj = ConvertPathToTrajectory(robot, path[::-1])
        if traj != None:
            robot.GetController().SetPath(traj)
        waitrobot(robot)

        #grid.printme()
        previous = [grid.start_n.x, grid.start_n.y, grid.start_n.theta]
        count += 1
        with env:
            flag, changed_n = checkCollision(robot, env)
            if flag:
                # if there is edge value changed:
                k_m += heuristic(last_n, grid.start_n)
                last_n = grid.start_n
                ### TODO: update vertix
                for node in changed_n:
                    updateVertex(node)
                computeShortestPath()
        plotPoint(grid.start_n, 5, [0,0,0], 0.1)
        if count == 200:
            new_pose6[0][3] = 3
            table6.SetTransform(new_pose6)
        if count == 550:
            new_pose6[0][3] = 0
            table6.SetTransform(new_pose6)
            new_pose7[0][3] = 0
            new_pose7[1][3] = 1
            table7.SetTransform(new_pose7)
        if count == 800:
            new_pose4[0][3] = 1.5
            new_pose4[1][3] = -0.5
            table4.SetTransform(new_pose4)
            new_pose7[0][3] = 2.2
            new_pose7[1][3] = 0.3
            table7.SetTransform(new_pose7)

    print count
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

