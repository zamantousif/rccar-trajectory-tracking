import rospy
from std_msgs.msg import Float32MultiArray
import time
import sys
import signal
import numpy as np



if __name__ == '__main__':
	rospy.init_node("fakegoal")
	#oldTime = time.time()
	goal = Float32MultiArray()

	# goal.data = [0,1.5,-3.14] # goal 1
	# goal.data = [0,-1.5,0.] # goal 2
	goal.data = [-1.5,0,np.pi] # goal 3


	pub = rospy.Publisher("/racer/map/goal",Float32MultiArray,queue_size = 1)

	while(True):
		try:
			
			key = raw_input("press Q to regenerate traj")
			if(key == 'q'):
				pub.publish(goal)
				#oldTime = time.time()
				#print time.time()
			#rospy.sleep(9.0)
		except KeyboardInterrupt:
			sys.exit(0)


	rospy.spin()