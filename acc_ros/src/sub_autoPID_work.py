#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

global initTime
global prev_PubRunTime
global kp
global ki
global kd
global desiredSpeed
global heartBeat
global const_power_test
global runTime
global estop
global stopFlag
global currentTime
global lastPeintTime
global printtime
global curSpeed
prev_PubRunTime = 0
kp = 1.5
ki = 0.0#0.001
kd = 0.5
desiredSpeed = 15
heartBeat = 0
const_power_test = 0
runTime = 1000
estop = 0
stopFlag = 1
lastPrintTime = 0


def getCurrentRunTime():
	global initTime
	return time.time()-initTime
	

def callbackSpeed(data):
	global lastPrintTime
	global printtime
	global curSpeed
	if getCurrentRunTime() - lastPrintTime > 0.1:
		printtime = True
		#print(str(getCurrentRunTime())+","+str(data.data))
		curSpeed = data.data
		lastPrintTime = getCurrentRunTime()
		PIDPublisher(False)
		pub_estop.publish(False)
		

def PIDPublisher(end):
	global kp
	global ki
	global kd
	global desiredSpeed
	global heartBeat
	global const_power_test
	global runTime
	global prev_PubRunTime
	global pub_pidinfo
	
	if 0 < getCurrentRunTime() < runTime and not end:
		heartBeat = 1
		# print("Running...")
	else:
		heartBeat = 0
	
	if getCurrentRunTime() - prev_PubRunTime >= 0.1:
		PIDVal = Float32MultiArray()
		PIDVal.data = [kp, ki, kd, desiredSpeed, heartBeat, const_power_test]
		pub_pidinfo.publish(PIDVal)
		prev_PubRunTime = getCurrentRunTime()
	

def emergencyStop():
	print "'on_shutdown' event check..."
	global stopFlag
	global pub_estop
	global runTime
	#print("estop")
	if getCurrentRunTime() < runTime and stopFlag == 1:
		#print "stopFlag in 'if' condition..."
		# stopTime = getCurrentRunTime()
		# while (getCurrentRunTime() - stopTime <= 2):
		PIDPublisher(True)
		pub_estop.publish(True)
		stopFlag = 0


	

def callbackNextSpeed(data):
	global desiredSpeed
	print "callbackNextSpeed..."
	desiredSpeed = data.data # / 0.1109
	print "Desired speed :: ",desiredSpeed
	PIDPublisher(False)
	pub_estop.publish(False)
	#desiredSpeed = 0



def run():
	global initTime
	global pub_estop
	global pub_pidinfo
	global printtime
	global curSpeed
	printtime = True
	curSpeed = 0.0
	rospy.init_node('PIDPublisher', anonymous=True)
	# print("Ready for 3 seconds...")
	rospy.sleep(0.1)	# 0.1 seconds for ready when the code is runnning
	# print("GO!")
	initTime = time.time()
	pub_estop = rospy.Publisher('racer/teensy/estop',Bool, queue_size = 1)
	pub_pidinfo = rospy.Publisher('racer/teensy/pidinfo', Float32MultiArray, queue_size=1)
	rospy.Subscriber('racer/teensy/rpm', Float32, callbackSpeed)
	rospy.Subscriber('racer/ACC/nextSpeed', Float32, callbackNextSpeed)
	#if rospy.is_shutdown():
	rospy.on_shutdown(emergencyStop)
	#while True:
	r = rospy.Rate(50)
	while not rospy.is_shutdown():
		#if printtime == True:
		#	print(str(getCurrentRunTime())+","+str(curSpeed))
	    	#printtime = False
		r.sleep()
	#rospy.spin()

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
	pass
