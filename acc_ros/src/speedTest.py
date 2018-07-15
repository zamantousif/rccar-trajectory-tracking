import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import time




if __name__ == '__main__':
    rospy.init_node("SpeedTest")
    pub = rospy.Publisher("/racer/ACC/nextSpeed", Float32, queue_size = 1)
    pubs = rospy.Publisher("/autonomous_rc/Steering", Float32, queue_size = 1)
    
    #count = 0
    
    while(True):
        #count+=1
        pub.publish(0.)

        #if(count-2*int(count/2) == 1):
         #   pubs.publish(10100)
        #else:
        #    pubs.publish(10100)
        rospy.sleep(2)
        
