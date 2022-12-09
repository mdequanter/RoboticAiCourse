#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

distance = 0.0

def callback(data):
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    twist = Twist()
    
    nrValues = (len(data.ranges))
    steps = (360/len(data.ranges))
 
    frontLeftMin = nrValues - 10
    frontRight = 10

    frontLeftDetector = min(tuple(filter(lambda item:item > 0.0, data.ranges[frontLeftMin:nrValues])))
    print (frontLeftDetector)
    frontRightDetector = min(tuple(filter(lambda item:item > 0.0, data.ranges[0:frontRight])))
    print (frontRightDetector)

    twist.linear.y = 0.0
    twist.linear.z = 0.0
    if ( frontLeftDetector > 0.3 and frontRightDetector > 0.3 ) :
        twist.linear.x = 0.1 
    else :
        twist.linear.x = 0
    
    pub.publish(twist)
    


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('scan', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
