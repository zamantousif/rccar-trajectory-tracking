#!/usr/bin/env python

import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import time
import csv

global oldTime

def quaternion_to_euler_angle(w, x, y, z):
	ysqr = y * y

	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))

	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))

	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))

	return X, Y, Z


def poseCallBack(data):
	global oldTime
	x = data.transform.translation.x
	y = data.transform.translation.y
	z = data.transform.translation.z
	rx = data.transform.rotation.x
	ry = data.transform.rotation.y
	rz = data.transform.rotation.z
	rw = data.transform.rotation.w

	combo = quaternion_to_euler_angle(rx,ry,rz,rw)
	freq = 1/(time.time()-oldTime)

	result = Float32MultiArray()
	result.data = [x,y,combo[0]]

	if(time.time()-oldTime > 1):
		pub.publish(result)
		oldTime = time.time()

		with open ('CarTrackdata.csv', 'a') as f:
			writer = csv.writer(f)
			writer.writerow(result.data)
		print result.data
	#print np.degrees(combo)



if __name__ == '__main__':
	global oldTime

	oldTime = time.time()
	print "start Trans"
	rospy.init_node("poseCallBack")
	pub = rospy.Publisher("/vicon/Pose",Float32MultiArray,queue_size = 2)
	sub = rospy.Subscriber("/vicon/rccar_car_01/rccar_car_01",TransformStamped,poseCallBack)



	rospy.spin()
