

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32















if __name__ == '__main__':

	rospy.init_node('estop')
	pubSpeed = rospy.Publisher("/racer/ACC/nextSpeed", Float32, queue_size = 1)
	pubSteer = rospy.Publisher("racer/teensy/steer", Int32, queue_size = 1)

	while(True):
		pubSpeed.publish(0.0)
		pubSteer.publish(10130)
	rospy.spin()
	