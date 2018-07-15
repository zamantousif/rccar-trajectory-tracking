import rospy
import numpy as np 
import time
from std_msgs.msg import Float32MultiArray
import transforms3d
from geometry_msgs.msg import PoseStamped as PS
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path


def poseSub(data):
	global path
	global header
	global pub
	path.poses.append(data)
	pub.publish(path)
	header = data.header
	#print data
	



def updateGoal(data):
	global path
	global header
	path = Path()
	path.header = header
	print "regenerate", time.time()


def initMessage():
	global path 
	global pub
	rospy.init_node("carTr")
	path = Path()

	pub = rospy.Publisher('carTrack', Path, queue_size=0)
	sub = rospy.Subscriber('viconTracker', PS, poseSub)
	goalSub = rospy.Subscriber("/racer/map/goal", Float32MultiArray,updateGoal)
	rospy.spin()


if __name__ == '__main__':


	try:
		initMessage()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start CarPoseTrack node.')
