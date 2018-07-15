#!/usr/bin/env python

import rospy
import numpy
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

global tototaimu
global counts

global increment

def callback(data):
    global pub
    global tototaimu
    global counts
    global filt
    global minAng
    global pub
    resultSet = []
    resultSet.append(8.0)
    #print len(data.ranges)
    #print data.range_min
    for i in range(0,721):
        if(data.ranges[i]<filt[i]):
            resultSet.append(abs(data.ranges[i]*math.cos(minAng+increment*float(i))))
    res = min(resultSet)       
    print "Distance to closest point :: ",res
    
    pub.publish(res)
    
    

    
	
def offsetDis(dis):
    return -0.0393*dis*dis+0.4448*dis+0.1797
    



def listener():
    global pub
    #print "!!!"
    rospy.Subscriber("/scan",LaserScan,callback)
    pub = rospy.Publisher("/racer/lidar/distance", Float32, queue_size = 1)
    #print "in listener"
    rospy.spin()
	
    	

def init_values():
    global increment
    global filt
    global minAng
    
    maxAng = 1.56466042995
    minAng = -1.56466042995
    increment = 0.00436332309619
    
    zone = 0.35
    
    
    filt = []
    for i in range(0,721):
        ang = math.sin(minAng+increment*float(i))
        if (ang != 0.0):
            dis = abs((zone/ang))
        else:
            dis = 12.0
        filt.append(min([12.0,dis]))
        #print dis
        

   
    #print filt



if __name__ == '__main__':
    global tototaimu
    global counts
    init_values()
    tototaimu = time.time()
    counts = 0
    rospy.init_node('readDistance_node')
    
    listener()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
        
	
