#!/usr/bin/env python
# -*- coding: utf-8 -*-
#HW3 for EECS 598 Motion Planning
import time
import openravepy
from openravepy import *
import numpy
from scipy import spatial
import random
#### YOUR IMPORTS GO HERE ####
RaveInitialize()
RaveLoadPlugin('build/myplugin')
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


def GetEETransform(robot,activedofvalues=None):
    #if activedofvalues != None:
    #    robot.SetActiveDOFValues(activedofvalues);
    robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()


def draw_raw_path(path,robot,handle):
    for i in range(path.shape[0]):
        dof=path[i]
        tf=GetEETransform(robot,dof)
        handles.append(env.plot3(points=array(((tf[0][3],tf[1][3],tf[2][3]))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))

    return

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [joint1_i, joint2_i, joint3_i,...]

    if path==[]:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeActiveDOFTrajectory(traj,robot)#,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj



if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()        
    # load a scene from ProjectRoom environment XML file
    env.Load('hw3.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    ### INITIALIZE YOUR PLUGIN HERE ###
    MyNewModule = RaveCreateModule(env,'mynewmodule')

    ### END INITIALIZING YOUR PLUGIN ###
   

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);
  
    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])      
    startconfig = [-0.15,0.075,-1.008,0,0,-0.11,0]
    robot.SetActiveDOFValues(startconfig);
    robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)
    handles = []
    with env:
        goalconfig = [0.449,-0.201,-0.151,0,0,-0.11,0]
        ### YOUR CODE HERE ###
        ###call your plugin to plan, draw, and execute a path from the current configuration of the left arm to the goalconfig

        bias=10
        step_size=0.05 
        max_steps=30   
        goalconfig.append(bias)
        goalconfig.append(step_size)
        input_data=str(max_steps)+"|"+str(bias)+"|"+str(step_size)
        for it in goalconfig:
            input_data=input_data+"|"+str(it)
        start = time.clock()

        out_check=MyNewModule.SendCommand('rrt '+input_data) # rrt connect
        #out_check=MyNewModule.SendCommand('rrtextend '+input_data) # rrt extend
       
        print out_check
        end = time.clock()
        print "Time counted by python : ", end - start
        ### END OF YOUR CODE ###
    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()

