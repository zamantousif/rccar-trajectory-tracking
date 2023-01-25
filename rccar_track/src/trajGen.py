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


def initValues():

	global ts #timestep
	global unitSpeed
	global start
	global goal
	global v_final
	global xs
	global ys
	ts= 1
	rps2mps = 0.1109
	ms2rps = 1./rps2mps
	x = 0
	y = 0
	theta = 0
	unitSpeed = 0.5
	start = [x,y,theta]
	goal = [10.,10.,0.]
	v_final = 0.3
	l = 0.34
	cc2 = 0.15

def initMessages():

	global pubPath
	global pubTrajCoeff
	rospy.init_node('TrajectoryTracking')
	print "Initialized TrajectoryTracking"
	pubTrajCoeff = rospy.Publisher("/racer/TrajGen", Float32MultiArray, queue_size = 1)
	poseSub = rospy.Subscriber("/vicon/Pose",Float32MultiArray,updatePose)
	#headerSub = rospy.Subscriber("/vicon/rccar_car_01/rccar_car_01",TransformStamped, headersub)
	goalSub = rospy.Subscriber("/racer/map/goal", Float32MultiArray,updateGoal)
	rospy.spin()


def updatePose(data):
	global x
	global y
	global theta
	pose = data.data
	x = pose[0]
	y = pose[1]
	theta = pose[2]/180.*math.pi

def updateGoal(data):
	global start
	global goal
	start = [x,y,theta]
	goal = data.data
	traj(start,goal)


def traj(start, goal):
	global ts
	global unitSpeed
	global xs
	global ys
	global t_init
	global header
	global pubPath
	global tf
	global v_final
	global Totalsteps
	global pubTrajCoeff

	v = 1.0
	t_init = time.time()
	t0 = 0.0
	tf = (((abs(start[0]-goal[0]))**2+(abs(start[1]-goal[1]))**2)**0.5)/unitSpeed
	#print "distance & time", tf*unitSpeed, tf

	if(tf<(0.001/unitSpeed)):
		xs = np.array([start[0],0.,0.,0.])
		ys = np.array([start[1],0.,0.,0.])
		print "reach goal xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
		ab=np.array([xs,ys,time.time(),0,0])
		pubTrajCoeff.publish(ab)
		return xs, ys, time.time()
		pass

	A = np.array([[1.,t0,t0**2,t0**3,0.,0.,0.,0.],\
				[0.,0.,0.,0.,1.,t0,t0**2,t0**3],\
				[1.,tf,tf**2,tf**3,0.,0.,0.,0.],\
				[0.,0.,0.,0.,1.,tf,tf**2,tf**3],\
				[0.,1.*sin(start[2]),2.*t0*sin(start[2]),3.*t0**2*sin(start[2]),0.,-1.*cos(start[2]),-2.*t0*cos(start[2]),-3.*t0**2*cos(start[2])],\
				[0.,1.*sin(goal[2]),2.*tf*sin(goal[2]),3.*tf**2*sin(goal[2]),0.,-1.*cos(goal[2]),-2.*tf*cos(goal[2]),-3.*tf**2*cos(goal[2])],\
				[0.,1.*cos(start[2]),2.*t0*cos(start[2]),3.*t0**2*cos(start[2]),0.,1.*sin(start[2]),2.*t0*sin(start[2]),3.*t0**2.*sin(start[2])],\
				[0.,1.*cos(goal[2]),2.*tf*cos(goal[2]),3.*tf**2*cos(goal[2]),0.,1.*sin(goal[2]),2.*tf*sin(goal[2]),3.*tf**2*sin(goal[2])]])

	b = np.array([start[0],\
				start[1],\
				goal[0],\
				goal[1],\
				0.0,\
				0.0,\
				v,\
				v_final])
	A = A.astype(np.float64)
	b = b.astype(np.float64)


	ab = np.linalg.solve(A,b)
	#print "ab value is ",ab
	xs = np.array([ab[0],ab[1],ab[2],ab[3]])
	ys = np.array([ab[4],ab[5],ab[6],ab[7]])

	Totalsteps = int(tf/ts)
	print "Totalsteps = ",Totalsteps
	trajGenData = Float32MultiArray()
	#trajGenData.data = [ab, t_init, tf , Totalsteps]
	trajGenData.data = [ab[0],ab[1],ab[2],ab[3],ab[4],ab[5],ab[6],ab[7],t_init,tf,Totalsteps]
	pubTrajCoeff.publish(trajGenData)
	#print "Done with trajectory generation"



if __name__ == "__main__":
	global rps2mps
	global ms2rps
	global ts #timestep
	global x
	global y
	global theta
	global w
	global v
	global unitSpeed
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
