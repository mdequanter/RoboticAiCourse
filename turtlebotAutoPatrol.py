#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

distance = 0.0

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

def goBackward() :
    twist = Twist()
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.linear.x = -0.1
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    twist.angular.x = 0.0    
    pub.publish(twist)
    


def goForward() :
    twist = Twist()
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.linear.x = 0.1
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    twist.angular.x = 0.0
    pub.publish(twist)

def turnRight() :
    twist = Twist()
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.linear.x = 0.05
    twist.angular.y = 0.0
    twist.angular.z = -0.4
    twist.angular.x = 0.0
    pub.publish(twist)


def turnLeft() :
    twist = Twist()
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.linear.x = 0.05
    twist.angular.y = 0.0
    twist.angular.z = 0.4
    twist.angular.x = 0.0
    pub.publish(twist)


def stop() :
    twist = Twist()
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.linear.x = 0.0
    pub.publish(twist)





def callback(data):
    
    nrValues = (len(data.ranges))
    steps = (360/len(data.ranges))
 
    frontLeftMin = nrValues - 16
    frontRight = 16

    frontLeftDetector = min(tuple(filter(lambda item:item > 0.0, data.ranges[frontLeftMin:nrValues])))
    print ("L:" + str(frontLeftDetector))
    frontRightDetector = min(tuple(filter(lambda item:item > 0.0, data.ranges[0:frontRight])))
    print ("R:" + str(frontRightDetector))


    if (frontLeftDetector < 0.2 or frontRightDetector < 0.2) :
        goBackward()
        print ("S")
 
    if ( frontLeftDetector > 0.4 and frontRightDetector > 0.4 ) :
        goForward() 
        print ("F")
    if (frontLeftDetector < 0.4  or frontRightDetector < 0.4) :
        print ("check" + str(frontLeftDetector) + "----" + str(frontRightDetector))
        if (frontLeftDetector >  frontRightDetector)  :
            turnRight()
            print ("R")
        if (frontLeftDetector <  frontRightDetector) :
            turnLeft()
            print ("L")
    


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
