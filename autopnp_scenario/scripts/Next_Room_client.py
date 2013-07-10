#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import smach
import smach_ros
import cv
import numpy as np

import actionlib
from nav_msgs.msg import OccupancyGrid 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import autopnp_scenario.msg


    
def NextRoom( InputImg, RoomCenterX, RoomCenterY , mapResolution , mapOriginX , mapOriginY ):  
    
    client = actionlib.SimpleActionClient( 'Find_Next_Room', autopnp_scenario.msg.Next_RoomAction )

    rospy.loginfo("Waiting for the Next Room Action server to start.")

    client.wait_for_server()    

    rospy.loginfo("Next Room Action server started, sending goal.")    
      
    goal = autopnp_scenario.msg.Next_RoomGoal( input_img = InputImg, 
                                               room_center_x = RoomCenterX, 
                                               room_center_y = RoomCenterY,
                                               map_resolution = mapResolution,
                                               Map_Origin_x = mapOriginX,
                                               Map_Origin_y = mapOriginY )
    
    client.send_goal(goal)
    
    finished_before_timeout = client.wait_for_result()
    if finished_before_timeout:
        state = client.get_state()
        if state is 3:
            state = 'SUCCEEDED'
            rospy.loginfo("Action finished: %s " % state)
        else:
            rospy.loginfo("Action finished: %s " % state)

    else:
        rospy.loginfo("Action did not finish before the time out.")
        
    return client.get_result()     