#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy

#### YOUR IMPORTS GO HERE ####
from scipy.spatial.transform import Rotation as R ##need installasion
from heapq import*
from pqdict import pqdict
import math
import matplotlib.pyplot as plt
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
    if path==[]:
	    return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')	
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
	    traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

def h(q,goalconfig,variant=4):
    #q_i = [x_i, y_i, theta_i]
    ## variant : decide which variant of A*
        # 1 : 4-connected space w/ manhattan distance heuristic
        # 2 : 4-connected space w/ euclidian distance heuristic
        # 3 : 8-connected space w/ manhattan distance heuristic
        # 4 : 8-connected space w/ euclidian distance heuristic
    diff=[goalconfig[0]-q[0],goalconfig[1]-q[1],goalconfig[2]-q[2]]
    if numpy.absolute(diff[2])>numpy.pi:
        diff[2]=numpy.pi*2-numpy.absolute(diff[2])
    if variant==1 or variant==3:
        ans=0
        for i in range(3):
            ans+=numpy.absolute(diff[i])
        return ans
    else:
        return numpy.linalg.norm(diff)

def getstr(point):
    ## from list to string
    x=point[0]
    y=point[1]
    t=point[2]
    key=str(x)+","+str(y)+","+str(t)
    return key

def getpt(key):
    ## from string to list
    cord=key.split(",")
    x=float(cord[0])
    y=float(cord[1])
    t=float(cord[2])
    return [x,y,t]

def simplecord(point,xy_step=0.1,theta_step=pi/10):
    ## example: from [0.1, 0.7, 3.14] to [1, 7, 10]
    ## In order to make converting from list to string easier
    ans=[point[0]/xy_step,point[1]/xy_step,point[2]/theta_step]
    return ans 

def complexcord(point,xy_step=0.1,theta_step=pi/10):
    ## example: from [1, 7, 10] to [0.1, 0.7, pi] 
    ## retrieve the correct pose
    ans=[point[0]*xy_step,point[1]*xy_step,point[2]*theta_step]
    return ans 

def backtrack(ParentMap,point,xy_step=0.1,theta_step=pi/10):
    # point is an interger list with simplified coordinate
    # key=getstr(point), string such as "10,10,5"
    # point=getpt(key), array such as [10,10,5]
    path=numpy.array(complexcord(point,xy_step,theta_step))
    path=resize(path,(1,3))
    key=getstr(point)
    while key in ParentMap:
        key=ParentMap[key]
        point=getpt(key)
        b=numpy.array(complexcord(point,xy_step,theta_step))
        b=resize(b,(1,3))
        path=numpy.concatenate((b,path), axis=0)
    return path

def find_neighbor(variant=4):
    ## variant : decide which variant of A*
        # 1 : 4-connected space w/ manhattan distance heuristic
        # 2 : 4-connected space w/ euclidian distance heuristic
        # 3 : 8-connected space w/ manhattan distance heuristic
        # 4 : 8-connected space w/ euclidian distance heuristic
    if variant==1 or variant==2:
        ans=numpy.zeros((6,3))
        for i in range(3):
            ans[2*i][i]=1
            ans[2*i+1][i]=-1
        return ans
    else :
        ans=numpy.zeros((3*3*3,3))
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    ans[k+j*3+i*3*3][2]=k-1
                    ans[k+j*3+i*3*3][1]=j-1
                    ans[k+j*3+i*3*3][0]=i-1
        ans=numpy.delete(ans, 13, 0)          
        return ans

def find_neighbor_dist(neighbor_matrix,variant=4,xy_step=0.1,theta_step=pi/10):
    ## variant : decide which variant of A*
        # 1 : 4-connected space w/ manhattan distance heuristic
        # 2 : 4-connected space w/ euclidian distance heuristic
        # 3 : 8-connected space w/ manhattan distance heuristic
        # 4 : 8-connected space w/ euclidian distance heuristic
    l=neighbor_matrix.shape[0]
    ans=numpy.zeros(l)
    for i in range(l):
        if variant==1 or variant==3:
            ans[i]=xy_step*(numpy.absolute(neighbor_matrix[i][0])+numpy.absolute(neighbor_matrix[i][1]))+numpy.absolute(neighbor_matrix[i][2])*theta_step
        else:
            diff=[numpy.absolute(neighbor_matrix[i][0])*xy_step,numpy.absolute(neighbor_matrix[i][1])*xy_step, (numpy.absolute(neighbor_matrix[i][2])*theta_step)]
            ans[i]=numpy.linalg.norm(diff)
    return ans

def check_collision_openrave(robot,env,neighbor_point,xy_step=0.1,theta_step=pi/10):
    point=complexcord(neighbor_point,xy_step,theta_step)
    robot.SetTransform([[numpy.cos(point[2]),-numpy.sin(point[2]),0,point[0]],[numpy.sin(point[2]),numpy.cos(point[2]),0,point[1]],[0,0,1,0.05],[0,0,0,1]])
    ans=env.CheckCollision(robot)
    return ans

def openrave_draw(handles,collision,neighbor_point,xy_step=0.1,theta_step=pi/10):
    point=complexcord(neighbor_point,xy_step,theta_step)
    if collision==True:
        handles.append(env.plot3(points=array(((point[0],point[1],0.05))),
                                   pointsize=5.0,
                                   colors=array(((1,0,0)))))
    else :
        handles.append(env.plot3(points=array(((point[0],point[1],0.05))),
                                   pointsize=5.0,
                                   colors=array(((0,0,1)))))
    return 

def e(goalconfig,point,key,G,gmap,xy_step,theta_step):
    dem=h(complexcord(point,xy_step,theta_step),goalconfig)
    num=G-gmap[key]
    if numpy.allclose(numpy.array(dem),numpy.array(0)):
        return float('inf') 
    return num/dem

def ana(time_out,robot,goalconfig,env,variant=4,xy_step=0.1,theta_step=pi/10):

    start_time=time.clock()
    G = float('inf') 
    E = float('inf') 
    pose=poseFromMatrix(robot.GetTransform())
    r = R.from_quat(pose[0:4])
    point=[pose[4],pose[5],r.as_euler('zyx', degrees=False)[0]] 
    point=simplecord(point,xy_step,theta_step)
    key=getstr(point)
    ParentMap={}
    gmap={}
    gmap[key]=0

    pq_1 = pqdict({key:h(complexcord(point,xy_step,theta_step),goalconfig)*10000+gmap[key]}, reverse=False)
    # pq_1 is for first iteration
    # get smallest h-value since G = INF
    # if there are 2 points have the same h-value, get the points with smaller g-value
    pq = pqdict({key:e(goalconfig,point,key,G,gmap,xy_step,theta_step)}, reverse=True)
    
    target=simplecord(goalconfig ,xy_step,theta_step)
    neighbor_matrix=find_neighbor(variant)
    neighbor_num=neighbor_matrix.shape[0]
    neighbor_dist=find_neighbor_dist(neighbor_matrix,variant,xy_step,theta_step)
    handles = []
    t_max=pi/theta_step
    handles.append(env.plot3(points=array(((goalconfig[0],goalconfig[1],0.05))),
                                   pointsize=5.0,
                                   colors=array(((0,1,0)))))


    ### FIRST ITERATION OF ANA/IMPROVE ###
    while pq:
        it=pq_1.popitem()
        key=it[0]
        e_=pq[key]
        pq.pop(key)
        if e_<E:
            E=e_

        point=getpt(key)
        if numpy.allclose(numpy.array(point),numpy.array(target)):
            G=gmap[key]
            break

        for i in range(neighbor_num):
            neighbor_point=[ point[0]+neighbor_matrix[i][0],point[1]+neighbor_matrix[i][1],point[2]+neighbor_matrix[i][2] ]
            if numpy.absolute(neighbor_point[2])>t_max:
                if neighbor_point[2]>0:
                    neighbor_point[2]=neighbor_point[2]-2*t_max
                else:
                    neighbor_point[2]=neighbor_point[2]+2*t_max
            collision=check_collision_openrave(robot,env,neighbor_point,xy_step,theta_step)
            openrave_draw(handles,collision,neighbor_point,xy_step,theta_step)
            if collision==True:
                continue
            temp_g=gmap[key]+neighbor_dist[i]
            neighbor_key=getstr(neighbor_point)
            if neighbor_key not in gmap or temp_g < gmap[neighbor_key]:
                ParentMap[neighbor_key]=key
                gmap[neighbor_key]=temp_g
                if temp_g + h(complexcord(neighbor_point,xy_step,theta_step),goalconfig)<G:
                    pq[neighbor_key]=e(goalconfig,neighbor_point,neighbor_key,G,gmap,xy_step,theta_step)
                    pq_1[neighbor_key]=h(complexcord(neighbor_point,xy_step,theta_step),goalconfig)*10000+temp_g
    #######

    sub_sol=G
    x=numpy.array(G)
    kick={"123"}
    kick.clear()
    for k in pq:
        point=getpt(k)
        if gmap[k]+h(complexcord(point,xy_step,theta_step),goalconfig)<G :
            pq[k]=e(goalconfig,point,k,G,gmap,xy_step,theta_step)
        else:
            kick.add(k)

    for k in kick:
        pq.pop(k)
    end = time.clock()
    y=numpy.array(end-start_time)
    if end-start_time>time_out:
        print "time out"
        return

    while pq:
        #### improve#####
        while pq:
            it=pq.popitem()
            key=it[0]
            e_=it[1] 
            if e_<E:
                E=e_
            point=getpt(key)
            if numpy.allclose(numpy.array(point),numpy.array(target)):
                G=gmap[key]
                break
            for i in range(neighbor_num):
                neighbor_point=[ point[0]+neighbor_matrix[i][0],point[1]+neighbor_matrix[i][1],point[2]+neighbor_matrix[i][2] ]
                if numpy.absolute(neighbor_point[2])>t_max:
                    if neighbor_point[2]>0:
                        neighbor_point[2]=neighbor_point[2]-2*t_max
                    else:
                        neighbor_point[2]=neighbor_point[2]+2*t_max
                collision=check_collision_openrave(robot,env,neighbor_point,xy_step,theta_step)
                openrave_draw(handles,collision,neighbor_point,xy_step,theta_step)
                if collision==True:
                    continue
                temp_g=gmap[key]+neighbor_dist[i]
                neighbor_key=getstr(neighbor_point)
                if neighbor_key not in gmap or temp_g < gmap[neighbor_key]:
                    ParentMap[neighbor_key]=key
                    gmap[neighbor_key]=temp_g
                    if temp_g + h(complexcord(neighbor_point,xy_step,theta_step),goalconfig)<G:
                        pq[neighbor_key]=e(goalconfig,neighbor_point,neighbor_key,G,gmap,xy_step,theta_step)

        #######
        sub_sol=G
        x=numpy.append(x,G)
        kick={"123"}
        kick.clear()
        for k in pq:
            point=getpt(k)
            if gmap[k]+h(complexcord(point,xy_step,theta_step),goalconfig)<G :
                pq[k]=e(goalconfig,point,k,G,gmap,xy_step,theta_step)
            else:
                kick.add(k)

        for k in kick:
            pq.pop(k)
        end = time.clock()
        y=numpy.append(y,end-start_time)
        if end-start_time>time_out:
            print "time out"
            break

    return backtrack(ParentMap,target,xy_step,theta_step),x,y


def draw_raw_path(path,robot,handle,r,g,b):
    for i in range(path.shape[0]):
        tf=path[i]
        handle.append(env.plot3(points=array(((tf[0],tf[1],0.05))),
                                   pointsize=5.0,
                                   colors=array(((r,g,b)))))

    return

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
    handle=[]

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])
        goalconfig = [2.6,-1.3,-pi/2]
        start = time.clock()
        ## decide which variant of A*
            # 1 : 4-connected space w/ manhattan distance heuristic
            # 2 : 4-connected space w/ euclidian distance heuristic
            # 3 : 8-connected space w/ manhattan distance heuristic
            # 4 : 8-connected space w/ euclidian distance heuristic
        variant=4  
        ## 
        xy_step=0.1
        theta_step=pi/6
        time_out=60*10
        output,xvals, yvals=ana(time_out,robot,goalconfig,env,variant,xy_step,theta_step)
        
        path = []
        if output!=[]:
            path=output
            draw_raw_path(path,robot,handle,0,0,0)
        else:
            print "ANA* Failure"
        ### end ###
        end = time.clock()
        print "Time: ", end - start
        traj = ConvertPathToTrajectory(robot, path)
        if traj != None:
            robot.GetController().SetPath(traj)
            

    waitrobot(robot)
    ## the performance of A*
    f_star=10.6949603604
    t_star=72.000835
    ##
    plt.figure(0)
    plt.plot(yvals, xvals,'b')
    x_=numpy.array([yvals[0],yvals[-1]])
    y_=numpy.array([xvals[0],xvals[-1]])
    plt.plot(x_, numpy.array([f_star,f_star]),'r')
    plt.plot(numpy.array([t_star,t_star]), y_,'r')
    plt.plot(t_star, f_star,'r*')
    plt.xlabel("Time (s)")
    plt.ylabel("error")
    plt.legend(['ANA*','A*'])
    plt.show()

    raw_input("Press enter to exit...")


