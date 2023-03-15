#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


positions = ()



def movebase_client(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    
    print ("x: " + str(x))
    print ("y: " + str(y))
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x =  x
    goal.target_pose.pose.position.y =  y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
    
        # Define a dictionary of predefined locations and their coordinates
        locations = {"1": "Center of map","2": "Charging station","3": "Hospital"}
        coordinates = {"1":[0.000,-0.2846],"2":[-1.1284,-1.4429],"3":[1.1,2.2]}

        # Prompt the user to select a location
        for key in locations.keys():
            print(key + " " +locations[key])
               
        selection = input("Select the new location:")

        
        print ("you selected:")
        print (locations[selection])
        
        rospy.init_node('movebase_client_py')
        result = movebase_client(coordinates[selection][0],coordinates[selection][1])
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
