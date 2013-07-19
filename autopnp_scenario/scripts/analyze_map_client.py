#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import cv
import numpy as np

import actionlib
from nav_msgs.msg import OccupancyGrid 

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import autopnp_scenario.msg

class AnalyzeMap():
    def __init__(self):
        
        rospy.Subscriber('/map', OccupancyGrid, self.update_map)
        
        self.map_ = None
        self.map_origin = None
        self.map_resolution = None
        
    def update_map(self, map_msg):
        
        self.map_resolution_ = map_msg.info.resolution
        
        self.map_origin_x = map_msg.info.origin.position.x
        self.map_origin_y = map_msg.info.origin.position.y
        
        self.map_origin = ( map_msg.info.origin.position.x , map_msg.info.origin.position.y )
        self.map_ = 255*np.ones(( map_msg.info.height , map_msg.info.width) , np.uint8 )
        
        i = 0
        for v in range(0,map_msg.info.height):
            for u in range(0,map_msg.info.width):
                if map_msg.data[i] != 0:
                    self.map_[v][u] = 0
                i += 1
                
        
    def Analyze_Map_client(self):  
                    
        mat = cv.fromarray(self.map_)
        #cv.ShowImage( "map_image", mat )
        #cv.WaitKey(10)
        
        client = actionlib.SimpleActionClient( 'Analyze_Map', autopnp_scenario.msg.AnalyzeMapAction )
               
        cv_image = CvBridge()
        
        goal = autopnp_scenario.msg.AnalyzeMapGoal( input_img = cv_image.cv_to_imgmsg( mat , "mono8") , 
                                                     map_resolution = self.map_resolution_ , 
                                                     Map_Origin_x = self.map_origin_x , 
                                                     Map_Origin_y = self.map_origin_y )
        
        rospy.loginfo("Waiting for the Analyze Map Action server to start.")
    
        client.wait_for_server()
        
        rospy.loginfo("Analyze Map Action server started, sending goal.")
        
        client.send_goal(goal)
        
        finished_before_timeout = client.wait_for_result(rospy.Duration(30.0))
        
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


#Only need, if you want to test the Analyze Map Action Server from Client 
if __name__ == '__main__':
    
    rospy.init_node('test_Analyze_Map')
    
    test = AnalyzeMap()
    
    rospy.sleep(10)
    
    test.Analyze_Map_client()
    
    rospy.spin()
    
    
    
