#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped as PS
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path

def callback(data):
    global pose
    global path
    global pubPath
    global cnt
    global pub
    header = data.header
    x = data.transform.translation.x
    y = data.transform.translation.y
    z = data.transform.translation.z

    rx = data.transform.rotation.x
    ry = data.transform.rotation.y
    rz = data.transform.rotation.z
    rw = data.transform.rotation.w

    pose.header=header
    pose.pose.position.x=x
    pose.pose.position.y=y
    pose.pose.position.z=z
    pose.pose.orientation.x=rx
    pose.pose.orientation.y=ry
    pose.pose.orientation.z=rz
    pose.pose.orientation.w=rw
    if(cnt==0):
        path.header=header
        cnt=1  
    path.poses.append(pose)
    print len(path.poses)
    pub.publish(pose)
    #pubPath.publish(path)
    



def talker():
    global pose
    global path
    global pubPath
    global cnt
    global pub
    cnt=0
    pose = PS()
    path = Path()
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('viconTracker', PS, queue_size=1)
    pubPath=rospy.Publisher('viconCarTrajectory',Path,queue_size=1)
    sub = rospy.Subscriber("/vicon/rccar_car_01/rccar_car_01",TransformStamped, callback) #data type??
    rospy.spin()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
