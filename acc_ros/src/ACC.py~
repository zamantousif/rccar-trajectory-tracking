#!/usr/bin/env python

import rospy
import numpy as np 
import math 
from scipy.constants import g as grav
from math import sqrt
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from cvxopt import matrix
from cvxopt.blas import dot
from cvxopt.solvers import qp
import cvxopt
import time
import csv

global m			#car mass
global g 			#9.81
global se 
global F0 
global F1 
global F2 
global gamma
global ca 
global cd 
global psc 			#soft constraint
global x2 			#car current speed
global v0 			#leading car speed
global vd 			#desire speed
global D_old 		#old distance to front barrier
global D 			#current distance to front barrier
global softness 	#softness factor
global initD 		#initial distance to front
global dt    		#time interval
global pub 
global preV
global preTime		#used for calculate time interval
global x_old

'''
def simuSpeed(data):
	global x2
	global rps2mps
	global lowSpeed
	x2 = data*rps2mps
'''
def updateSpeed(data):

	'''
	speed call back funciton
	'''
	global x2
	global rps2mps
	global lowSpeed
	
	x2 = data.data * 0.1109
	#print "x2", x2, data.data

	if(x2>1.199 and lowSpeed == True):			#high speed
		lowSpeed = False
		switchMode(lowSpeed)
	elif(x2<=1.199 and lowSpeed == False):		#low Speed		
		lowSpeed = True
		switchMode(lowSpeed)

#*************************************************************

def updateDistance(data):

	'''
	distance call back function
	'''
	#print "updateDis"
	#print "running ACC"
	global v0
	global D_old
	global D 
	global pub
	global preV
	global preTime
	global dt

	D_old = D						#distance update
	D = data.data
	
	
	currentTime = time.time()
	dt = currentTime-preTime		#find time interval
	
	if(dt<0.1):
		return 0
		pass
	
	#print "==========================================="
	preTime = currentTime
	v0 = distance2speed(D_old, D)
	#v0 = 0.5	
	#print "Frame Frequency:", 1/dt


	if(D<=1.0):
		pub.publish((0.0))#=======================
		#print "front car less than 1.0m BREAK!"
	else:
		ACC_cal()

#*************************************************************
#*************************************************************
#*************************************************************

def switchMode(speed):
	if (speed):			#low speed

		F0 = 0.0
		F1 = 0.0
		F2 = 36.0
		#print "switch to low speed mode========="
	else:				#high speed
		F0 = 5.805
		F1 = -13.914
		F2 = 44.3
		#print "switch to high speed mode@@@@@@@@@"

#*************************************************************

def upDateQueue(now):
	global preV
	#print "now", now
	'''
	if(now == 0.0):
		preV[0] = 0.0
		preV[1] = 0.0
		preV[2] = 0.0
		preV[3] = 0.0
		print "Speed:", 0.0
		return 0.0
	

	if(now<0.3 and now>0.0):
		#preV.append(now)
		preV.append(1.6*dt)
		preV.pop(0)
	else:
		preV.append(now)
		preV.pop(0)
	print preV
	
	result = 0.4*preV[3]+0.3*preV[2]+0.2*preV[1]+0.1*preV[0]
	
	print "Buffered Speed:", result

	#return result
	'''
	return now

#*************************************************************

def distance2speed(D1, D2):
	'''
	D2 is the new distance input

	D1 is the old distance input
	Output is the speed in dt time interval
	'''
	
	relativeSpeed = (D2-D1)/dt
	v0 = relativeSpeed + x2
	
	return v0
#*************************************************************

def ms2rpm(v):
	return v/rps2mps

#*************************************************************
#*************************************************************
#*************************************************************

def ACC_cal():


	'''
	calculate LCF and BCF using QP
	publish the next speed
	'''

	global m
	global g 
	global se 
	global F0 
	global F1 
	global F2 
	global gamma
	global ca 
	global cd 
	global psc 
	global x2
	global v0
	global vd 
	global D_old
	global D 
	global softness
	global initD 
	global dt
	global pub
	global x_old
	global pubSteer
	global initTime

	
	block = 3.5
	if (D <= 1.0):
		print "less than 1.0m"
		return 0.0
		pass
	print "======================================"
	z = D 
	y = x2 - vd
	#y = vd-x2
	#print "speed difference", x2, vd, y 
	#print "x2 and vd and y", x2, vd, y
	Fr = F0 + F1*(y+vd)+F2*(y+vd)**2
	Vv0 = -2.0*y/m*Fr + se*(y**2)
	
	Vv1 = 2.0*y/m
	
	#print "vv0, vv1", Vv0, Vv1
	h = z-block*x2-0.3					#0.3m for parking
	#h = z-block*x2
	try:		
		B = -1.0* math.log(h/(1+h))
	except:
		h = 0.0001
		B = -1.0* math.log(h/(1+h))

	LfB = -1.0*(block*Fr + m*(v0 - x2))/(m*(1.0 - block*x2 + z)*(-block*x2 + z));
	LgB = block/(m*(1.0 - block*x2 + z)*(-block*x2 + z))
	try:
		hf = -block*x2-0.5*((v0-x2)**2)/cd*g+z
		Bf = -1.0* math.log(hf/(1.0+hf))
	except:
		hf = 0.001
		Bf = -1.0* math.log(hf/(1.0+hf))
		#print "!!!!!!??????????!!!!"	
	'''	
	LfBf = (v0-x2-block)/(m*cd*g)/(-1.0*hf+hf**2)
	LgBf = (m*cd*g*(v0-x2)-Fr*(v0-x2-block))/(m*cd*g)/(-1.0*hf+hf**2)

	H = matrix([[2.0/(m**2), 0.0], [0.0, 2*psc]])		#P

	F = matrix([-2.0*Fr/(m**2), 0.0])					#q
	A = matrix([[Vv1, -1.0], [LgB, 0.0], [1.0,0.0], [1.0, 0.0], [LgBf, 0.0]]).T

	b = matrix([-1.0*Vv0, -1.0*LfB+gamma/B, vd/m*dt, ca*m*g, -1.0*LgBf+1.0/Bf])
	'''
	H = matrix([[2.0/(m**2), 0.0], [0.0, 2*psc]])		#P



	F = matrix([-2.0*Fr/(m**2), 0.0])					#q

	A = matrix([[Vv1, -1.0], [LgB, 0.0]]).T



	b = matrix([-1.0*Vv0,-1.0*LfB+gamma/B])
	#print A
	#print b

    
	
	#A = matrix([[Vv1, -1.0], [LgB, 0.0], [1.0, 0.0], [-1.0, 0.0], [LgBf, 0.0]]).T

	#b = matrix([-1.0*Vv0, -1.0*LfB+gamma/B, ca*m*g, cd*m*g, -1.0*LgBf+1.0/Bf])




	try:
		
		U = cvxopt.solvers.qp(H, F, A, b)
		#result = min(ca*g, max(-cd*g, (U["x"][0]/m)))
		result = (U["x"][0]-Fr)/m
		temp = [time.time()-initTime, x2, v0, D, result]
		#time, currentV, frontV, Dis, force
		with open ('/home/ubuntu/catkin_ws/src/ACC/accdata.csv', 'a') as f:
			writer = csv.writer(f)
			writer.writerow(temp)

		print temp
		
		#print "result", U["x"],result
        #m/s
		#print "result, result, result, result", result
	

	
	except:
		result = 0.0
		pub.publish(0.0)#========================
		print "oops Hit!!!XXXXXXXXXXXXXXXxxxxxxxxxxxxxXX"

		return result
		
	result = min(result/0.1109, 3.0)
	vel = (result*dt)+x2/0.1109 #rpm
	#print "vel",vel




	if (vel<0.0):
		pub.publish(0.0)#========================
		print "Backup Protection"

	elif(vel>(vd*1.1/0.1109)):
		vel = vd*1.1/0.1109
		#vel = 5.0/0.1109	
		pub.publish(vel)#=========================
		print "Speeding Protection"


	
	else:

		toPub = vel	
		x_old = toPub
		pubSteer.publish(10120)	
		if(x2 == 0 and toPub > 0):
		    pub.publish(3.0)
		else:
		    pub.publish(toPub)
		print "Moving Forward! Next speed:rpm", toPub
		#simuSpeed(toPub)
	
	
	return vel

#*************************************************************
#*************************************************************
#*************************************************************

def initialValues():

	'''
	boring initials
	'''

	global m
	global g 
	global se 
	global F0 
	global F1 
	global F2 
	global gamma
	global ca 
	global cd 
	global psc 
	global x2
	global v0
	global vd 
	global D_old
	global D 
	global softness
	global initD 
	global dt
	global preV
	global lowSpeed
	global rps2mps
	global preTime
	global x_old
	global initTime

    
    
	x_old = 0.0

	rospy.init_node('ACC')

	m = 6.0
	g = 9.81
	se = 10.0
	F0 = 0.1
	F1 = 0.5
	F2 = 0.125
	gamma = 1.0
	ca = 0.3
	cd = 0.3
	psc = math.exp(-5)
	lowSpeed = True
	rps2mps = 0.1109	

	preV = []
	preV.append(0.0)
	preV.append(0.0)
	preV.append(0.0)
	preV.append(0.0) 
	


	initD = 4.0
	vd = 2.0
	v0 = 0.0					#leading car speed
	x2 = 0.0					#current speed
	D = initD
	D_old = D
	softness = 1.0
	dt = 0.1					#initialized as 0.1s change with the time interval between each frame for the rest
	preTime = time.time()
	cvxopt.solvers.options['show_progress'] = False
	data = []
	initTime = time.time()

#*************************************************************
#*************************************************************
#*************************************************************

def runACC():
	global pub
	global pubSteer
	
	pub = rospy.Publisher("/racer/ACC/nextSpeed", Float32, queue_size = 2)

	pubSteer = rospy.Publisher("/racer/teensy/steer", Int32, queue_size = 1)
    
    
	speedSub = rospy.Subscriber("/racer/teensy/rpm", Float32, updateSpeed)

	distanceSub = rospy.Subscriber("/racer/lidar/distance", Float32, updateDistance)

	pubtest = rospy.Publisher("/testtest", Float32, queue_size = 1)
	
	
	rospy.spin()

#*************************************************************
#*************************************************************
#*************************************************************

if __name__ == '__main__':

	initialValues()
	try:
		runACC();
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start ACC node.')
