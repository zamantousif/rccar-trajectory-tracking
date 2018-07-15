#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import time
import sys
import signal
import numpy as np 

global current_pose
current_pose = []

def reached_goal(current_pose,goal):
	print "current,goal -- ",current_pose,goal
	d = np.sqrt((current_pose[0]-goal[0])**2+(current_pose[1]-goal[1])**2)
	# print "distance -- ",d
	print d
	if(d<.5):
		print "reached_goal"
		return 1
	else:
		return 0

def callback_pose(data):
	global current_pose
	current_pose = data.data


if __name__ == '__main__':
	rospy.init_node("fakegoal")
	#oldTime = time.time()

	# global current_pose

	goal = Float32MultiArray()

	goal1 = [0,0,0]
	goal2 = [1.5,0,np.pi/2]
	goal3 = [-1.5,0,0]
	ipflag = 1

	initial_goal = 1

	pub = rospy.Publisher("/racer/map/goal",Float32MultiArray,queue_size = 1)
	rospy.Subscriber("/vicon/Pose",Float32MultiArray,callback_pose)
	r = rospy.Rate(10)
	# key = raw_input("press Q to regenerate traj")
	# goal.data = goal1
	while not rospy.is_shutdown():
		# try:
		# print current_pose
		if initial_goal:
			print "Waiting for initial goal..."
			rviz_goal = rospy.wait_for_message('/move_base_simple/goal',PoseStamped)
			initx = rviz_goal.pose.position.x
			inity = rviz_goal.pose.position.y
			init_theta = rviz_goal.pose.orientation.z
			init_goal = [initx,inity,init_theta]
			goal.data = init_goal
			initial_goal = 0
		print reached_goal(current_pose,goal.data)
		if(reached_goal(current_pose,goal.data)):
			print "waiting for new goal..."
			rviz_goal = rospy.wait_for_message('/move_base_simple/goal',PoseStamped)
			# print "----------------"
			# print rviz_goal
			newx = rviz_goal.pose.position.x
			newy = rviz_goal.pose.position.y
			new_theta = rviz_goal.pose.orientation.z
			new_goal = [newx,newy,new_theta]
			goal.data = new_goal
		pub.publish(goal)
		# if(key == 'q'):
		# 	goal.data = goal1
		# 	pub.publish(goal)
		# elif(key == 'w'):
		# 	goal.data = goal2
		# 	pub.publish(goal)
		# elif(key == 'e'):
		# 	goal.data = goal3
		# 	pub.publish(goal)
				#oldTime = time.time()
				#print time.time()
			#rospy.sleep(9.0)
		#except KeyboardInterrupt:
		#	sys.exit(0)
		r.sleep()
