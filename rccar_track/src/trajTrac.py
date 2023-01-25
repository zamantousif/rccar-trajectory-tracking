#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
import sympy
from math import *
from sympy import *
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from scipy.optimize import linprog
import transforms3d
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
import csv
from cvxpy import *
#from transforms3d.euler import euler2quad

#pos, steering, speed

def updateSpeed(data):
    '''
    Speed call back function
    '''

    global rps2mps
    global v
    global w

    v = data.data * rps2mps


def updatePose(data):
    global x
    global y
    global theta
    global uold
    global xs
    global ys
    global t_init
    global tf
    global tc
    global Totalsteps
    global approach

    pose = data.data
    x = pose[0]
    y = pose[1]
    theta = pose[2]/180.*math.pi
    X = np.zeros((3,1))
    X[0,0]=x
    X[1,0]=y
    X[2,0]=theta
    print "Current pose = ", X
    unext = np.zeros((2, 1))

    try:
        xss = xs
        yss = ys
        if (tc>(Totalsteps)):
            xref,uref = refState(xss,yss,Totalsteps)
        else:
            xref,uref = refState(xss,yss,tc)

        unext= MPC(xref,uref,X)
        print "unext value = ", unext
        #print uold
        tc=tc+1
        print "Timestep = ", tc
    except NameError as e:
        print "send goal from RViz..."


def sat(a,b,c):
    if(a<b):
        return b
    elif(a>c):
        return c
    else:
        return a

def storeGoal(data):
    global getgoal
    getgoal = data.data

def headersub(data):
    global header
    header = data.header

def refState(xss,yss,T):
    global pubPoint
    global header
    global tc
    global ts
    global NX
    global NU
    global N
    global L
    NX=3
    NU=2
    xref = np.zeros((NX, N+1))
    uref = np.zeros((NU, N+1))

    for i in range(0, N):
        t=T*ts+ i*ts

        xref[0,i] = xss[0]+xss[1]*t+xss[2]*t**2+xss[3]*t**3
        xref[1,i] = yss[0]+yss[1]*t+yss[2]*t**2+yss[3]*t**3
        dxr = xss[1]+2.*xss[2]*t+3.*xss[3]*t**2
        dyr = yss[1]+2.*yss[2]*t+3.*yss[3]*t**2
        ddxr = 2.*xss[2]+6.*xss[3]*t
        ddyr = 2.*yss[2]+6.*yss[3]*t
        thetar = math.atan2(dyr,dxr)
        xref[2,i] =  thetar
        vr = dxr*math.cos(thetar) + dyr*math.sin(thetar)
        uref[1,i] = vr
        # calculation of reference steering angle
        #theta_dot = 1/(1+((dyr/dxr)**2))
        #delta = math.atan((L*theta_dot)/vr)
        #uref[0,i] = delta

        wr = 1./vr*(ddyr*math.cos(thetar) - ddxr*math.sin(thetar))
        uref[0,i] = wr
        print "iteration and timestep = ", i,tc
        # print "thetar, vr, delta = ",thetar,vr,delta
        print "thetar, vr, wr = ",thetar,vr,wr


    point=PointStamped()
    point.header=header
    point.point.x=xref[0,0]
    point.point.y=xref[1,0]
    point.point.z=0
    pubPoint.publish(point)

    return xref, uref


def reffState(xss,yss,t):

    global pubPoint
    global header
    xr = xss[0]+xss[1]*t+xss[2]*t**2+xss[3]*t**3
    yr = yss[0]+yss[1]*t+yss[2]*t**2+yss[3]*t**3
    dxr = xss[1]+2.*xss[2]*t+3.*xss[3]*t**2
    dyr = yss[1]+2.*yss[2]*t+3.*yss[3]*t**2
    ddxr = 2.*xss[2]+6.*xss[3]*t
    ddyr = 2.*yss[2]+6.*yss[3]*t
    thetar = math.atan2(dyr,dxr)
    vr = dxr*math.cos(thetar) + dyr*math.sin(thetar)
    wr = 1./vr*(ddyr*math.cos(thetar) - ddxr*math.sin(thetar))

    return xr,yr,thetar,wr,vr

def updateTraj(data):
    global ts
    global xs
    global ys
    global t_init
    global header
    global pubPath
    global tf
    global v_final
    global Totalsteps
    global tc

    start = [x,y,theta]
    if tc == 0:
        ab = data.data
    else:
        return 0
        pass
    xs = np.array([ab[0],ab[1],ab[2],ab[3]])
    ys = np.array([ab[4],ab[5],ab[6],ab[7]])

    t_init = int(ab[8])
    tf = int(ab[9])
    Totalsteps = int(ab[10])


    path = Path()
    path.header = header


    for i in range(0,Totalsteps+1):
        #print i
        #print "start", start
        px,py,ptheta,pw,pv = reffState(xs, ys, float(i)*ts)
        quad = transforms3d.euler.euler2quat(0.,0.,ptheta)
        pos = PoseStamped()
        pos.header = header
        pos.pose.position.x=px
        pos.pose.position.y=py
        pos.pose.position.z=0.
        pos.pose.orientation.x=quad[1]
        pos.pose.orientation.y=quad[2]
        pos.pose.orientation.z=quad[3]
        pos.pose.orientation.w=quad[0]
        path.poses.append(pos)

        with open ('Trajdata.csv', 'a') as f:
            writer = csv.writer(f)
            writer.writerow([px, py, ptheta])

        pubPath.publish(path)

        examx = xs[0]+xs[1]*tf+xs[2]*tf**2+xs[3]*tf**3
        examy = ys[0]+ys[1]*tf+ys[2]*tf**2+ys[3]*tf**3

    return xs, ys, t_init

def euler_angle_to_quaternion(theta):

    return xr,yr,zr,wr

def linearize_state_space(v_ref, theta_ref, delta_ref):

    global ts
    global Lr
    global Lf

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[0, 2] = - ts * v_ref* math.sin(theta_ref)
    A[1, 2] = ts * v_ref * math.cos(theta_ref)

    B = np.zeros((NX, NU))
    B[0,1]= ts * math.cos(theta_ref)
    B[1,1]= ts * math.sin(theta_ref)
    B[2, 0] = ts*v_ref*(1+math.tan(delta_ref)**2)*(1/(Lf+Lr))
    B[2, 1] = ts* math.tan(delta_ref)*(1/(Lf+Lr))

    return A, B

# function to control steering angle and velocity using MPC

def MPC(xref, uref, xbar):
    global NX
    global NU
    global N
    global R
    global Q
    global Qf
    global x_max
    global x_min
    global u_min
    global u_max
    global tc
    global getgoal

    print "Inside MPC"
    start = time.time()
    xv = cvxpy.Variable((NX, N + 1))
    u = cvxpy.Variable((NU, N))
    cost = 0.0
    constraints = []
    u_min = np.array([[-1], [0]])
    u_max = np.array([[1], [1]])
    x_min = np.array([[-1.7], [-2.5], [-1]])
    x_max = np.array([[2], [2.8], [1]])
    # Gain matrices for MPC
    R = 2*np.diag([3.0, 1])  # input cost matrix
    Q = 5*np.diag([1.0, 1.0, 1.0])  # state cost matrix
    Qf = 2*np.diag([1.0, 1.0, 3.0])  # state final matrix
    print "Going to define cost function now"
    for k in range(0, N):
        if k != N:
            cost += cvxpy.quad_form(uref[:, k] - u[:, k], R)
            constraints += [cvxpy.abs(u[:,k]) <= u_max[:,0]]
        if k != 0:
            cost += cvxpy.quad_form(xref[:, k] - xv[:, k], Q)
            constraints += [cvxpy.abs(xv[:,k]) <= x_max[:,0]]
        A, B = linearize_state_space(uref[1, k], xref[2, k], uref[0, k])
        constraints += [xv[:, k + 1] == A * xv[:, k] + B * u[:, k]]
    cost += cvxpy.quad_form(xref[:, N] - xv[:, N], Qf)

    print "Defining constraints now"
    constraints += [xv[:, 0] == xbar[:,0]]

    print "Constraints are now defined"
    print "Going to solve MPC"
    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    # Solve the MPC problem if not solved already
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    print "Problem solving done. Answer is ", xv.value, u.value
    print "Problem status is ", prob.status
    end = time.time()
    print "Time taken in approach = ", end-start

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = xv.value[0, :]
        oy = xv.value[1, :]
        otheta = xv.value[2, :]
        odelta = u.value[0, 0]
        ov = u.value[1, 0]
    	print "Optimal value found"
    else:
        print("Error: Cannot solve mpc..")
        #ox, oy, otheta, odelta, ov = None, None, None, None, None, None
        ox, oy, otheta, odelta, ov = None, None, None, None, 0, 0

    unext = np.zeros((2,1))

    if tc == 0:
        odelta = uref[0,0]

    #unext[0,0]= uref[0,0]
    unext[0,0]= odelta
    unext[1,0]= ov
    #unext[0,0]=u.value[0,0]
    #unext[1,0]=u.value[1,0]

    print ("Optimal value from MPC is: steer, velocity "), unext[0,0],unext[1,0]
    #steer = math.atan(l/(v+0.1)*unext[0]) #check the validity of this line
    steer = unext[0]
    nextSpeed = unext[1]*ms2rps
    nextSpeed = nextSpeed[0]

    # if(problemSolved == True):
    #     steer = 0
    #     nextSpeed = 0

    print "xref0 xref1 xref2", xref[:,0], xref[:,1], xref[:,2]

    if((((xbar[0,0]-getgoal[0])**2+(xbar[1,0]-getgoal[1])**2)**0.5)<0.05):
        print "Nearing goal...Slowing down"
        steer = 0
        nextSpeed = 0

    print "Final result = ", steer/3.1415*180, nextSpeed


    #print (((x-x_r)**2+(y-y_r)**2)**0.5), abs(theta_r-theta)
    #w = v/l*math.tan(steer)

    # limits on control inputs for safety
    steer = sat(steer, -1.0,1.0)
    nextSpeed = sat(nextSpeed, 0.0,1.0)

    print "Limited value of steer = ", steer
    pwmSteer = int((steer+1.0)/2*(12207-8053)+8053)   #10130
    #st = sat(pwmSteer,8053,12207)
    st = pwmSteer
    print "PWM steer = ", st
    print "Next speed = ", nextSpeed
    #print "Speed & steer", nextSpeed, steer/3.1415*180.
    # if (nextSpeed<0.1):
    # 	nextSpeed = 0.0
    pubSteer.publish(st)
    pubSpeed.publish(nextSpeed)

    #print "steer", steer, nextSpeed
    print "================================================="
    return unext


def initMessages():
    global pubSpeed
    global pubSteer
    global pubPath
    global pubPoint
    #global trajSub
    rospy.init_node('TrajectoryTracking')
    print "Initialized TrajectoryTracking"
    pubSpeed = rospy.Publisher("/racer/ACC/nextSpeed", Float32, queue_size = 0)
    pubSteer = rospy.Publisher("racer/teensy/steer", Int32, queue_size = 1)
    pubPath = rospy.Publisher("/vicon/Path", Path, queue_size = 0)
    pubPoint = rospy.Publisher("/vicon/Point", PointStamped, queue_size = 0)

    speedSub = rospy.Subscriber("/racer/teensy/rpm", Float32, updateSpeed)
    #poseSub = rospy.Subsciber("/racer/map/pose",Float32MultiArray,updatePose)
    poseSub = rospy.Subscriber("/vicon/Pose",Float32MultiArray,updatePose)
    trajSub = rospy.Subscriber("/racer/TrajGen", Float32MultiArray,updateTraj)

    headerSub = rospy.Subscriber("/vicon/rccar_car_01/rccar_car_01",TransformStamped, headersub)
    getgoalSub = rospy.Subscriber("/racer/map/goal", Float32MultiArray,storeGoal)
    # Stop the car and keep straight
    pubSteer.publish(10130)
    pubSpeed.publish(0)
    rospy.spin()




def initValues():
    global rps2mps
    global ms2rps
    global ms2rps
    global ts #timestep
    global x
    global y
    global theta
    global w
    global v
    global tc #current time step

    global start
    global goal
    global v_final

    global xs
    global ys
    global l

    global uold

    global cc2
    global N
    global approach
    global v_max
    global v_min
    global delta_max
    global delta_min
    global x_min
    global x_max
    global u_min
    global u_max
    global Lf
    global Lr
    global L
    Lf = 0.195
    Lr = 0.145
    L = Lf + Lr
    N = 2
    rps2mps = 0.1109
    ms2rps = 1./rps2mps
    x = 0
    y = 0
    theta = 0
    w = 0
    v = 0
    start = [x,y,theta]
    goal = [10.,10.,0.]
    v_final = 0.3
    l = 0.34
    tc = 0
    ts = 1
    uold = [0.000000001,0.0000000001]
    cc2 = 0.15


if __name__ == "__main__":
    global rps2mps
    global ms2rps
    global ts #timestep
    global x
    global y
    global theta
    global w
    global v
    global tc #current time step
    global start
    global goal
    global v_final
    global xs
    global ys
    global uold

    initValues()

    try:
        initMessages()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start TrajectoryTracking node.')
