#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import radians, copysign, sqrt, pow, pi
import PyKDL


class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.summit_vel_publisher = rospy.Publisher('/summit_xl_control/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber(
            '/kobuki/laser/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/hokuyo_base/scan', LaserScan, self.summit_laser_callback)
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.get_rotation)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.summit_laser_msg = LaserScan()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.base_frame = '/base_link'
        self.angular_tolerance = radians(2)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            summit_connections = self.summit_vel_publisher.get_num_connections()
            if connections > 0 or summit_connections > 0:
                self.vel_publisher.publish(self.cmd)
                self.summit_vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
        

    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s

    def get_odom(self):

        # Get the current transform between the odom and base frames
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot)  = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))


    def get_rotation (self,msg):
        global roll, pitch, yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw)  = euler_from_quaternion (orientation_list)

    def rotate(self, degrees):
    	print (degrees)
    	kp=0.5
    	target_rad = degrees * 3.1415926535/180
    	self.cmd.angular.z = kp * (target_rad-self.yaw)
    	print (self.cmd.angular.z)
    	i=0
    	while (i <= 1):
    	    # Publish the velocity
    	    self.vel_publisher.publish(self.cmd)
    	    self.summit_vel_publisher.publish(self.cmd)
    	    i += 0.1
    	    print (self.yaw)
    	    self.rate.sleep()
    	self.stop_robot()
    	print ("done")
    	
    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]
        
    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == '__main__':
    #rospy.init_node('robot_control_node', anonymous=True)
    robotcontrol_object = RobotControl()
    try:
        #robotcontrol_object.move_straight_time('forward',0.1,1)
        #robotcontrol_object.turn('clockwise',0.1,1)
        #robotcontrol_object.turn('counterclockwise',0.1,1)
        robotcontrol_object.rotate(-60.0)

    except rospy.ROSInterruptException:
        pass
