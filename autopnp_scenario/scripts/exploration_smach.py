#!/usr/bin/env python

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import smach
import smach_ros

from map_segmentation_action_client import MapSegmentationActionClient
from find_next_unprocessed_room_action_client import find_next_unprocessed_room
from to_location_client import to_goal
from random_location_client import random_Location
from inspect_room_client import inspect_Room

# The AnalyzeMap class defines a state machine of smach which basically 
# call the map segmentation action client object and execute the function
# of action client to communicate with action server
class AnalyzeMap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['list_of_rooms'], output_keys=['analyze_map_data_img_',
                                                                            'analyze_map_data_map_resolution_',
                                                                            'analyze_map_data_map_origin_x_',
                                                                            'analyze_map_data_map_origin_y_',
                                                                            'analyze_map_data_room_center_x_', 
                                                                            'analyze_map_data_room_center_y_',
                                                                            'analyze_map_data_room_min_x_',
                                                                            'analyze_map_data_room_max_x_',
                                                                            'analyze_map_data_room_min_y_',
                                                                            'analyze_map_data_room_max_y_'])

    def execute(self, userdata):
        
        map_segmentation_action_client_object_ = MapSegmentationActionClient()
        
        rospy.sleep(1)
        rospy.loginfo('Executing state analyze map.....')
                        
        map_segmentation_action_server_result_ = map_segmentation_action_client_object_.map_segmentation_action_client_()   
        
        userdata.analyze_map_data_img_ = map_segmentation_action_server_result_.output_map
        userdata.analyze_map_data_map_resolution_ = map_segmentation_action_server_result_.map_resolution        
        userdata.analyze_map_data_map_origin_x_ = map_segmentation_action_server_result_.map_origin_x
        userdata.analyze_map_data_map_origin_y_ = map_segmentation_action_server_result_.map_origin_y        
        userdata.analyze_map_data_room_center_x_ = map_segmentation_action_server_result_.room_center_x_in_pixel
        userdata.analyze_map_data_room_center_y_ = map_segmentation_action_server_result_.room_center_y_in_pixel
        userdata.analyze_map_data_room_min_x_ = map_segmentation_action_server_result_.room_min_x_in_pixel
        userdata.analyze_map_data_room_max_x_ = map_segmentation_action_server_result_.room_max_x_in_pixel
        userdata.analyze_map_data_room_min_y_ = map_segmentation_action_server_result_.room_min_y_in_pixel        
        userdata.analyze_map_data_room_max_y_ = map_segmentation_action_server_result_.room_max_y_in_pixel
        
        return 'list_of_rooms'  
       
# The NextUnprocessedRoom class defines a state machine of smach which basically 
# call the find next unprocessed room action client object and execute the function
# of action client to communicate with action server
class NextUnprocessedRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['location','no_rooms','arrived'],input_keys=['next_unprocessed_room_data_img_',
                                                                                          'analyze_map_data_map_resolution_',
                                                                                          'analyze_map_data_map_origin_x_',
                                                                                          'analyze_map_data_map_origin_y_',
                                                                                          'analyze_map_data_room_center_x_',
                                                                                          'analyze_map_data_room_center_y_',
                                                                                          'F_N_R_room_number_in',
                                                                                          'F_N_R_loop_counter_in'],                             
                                                                             output_keys=['next_unprocessed_room_number_out',
                                                                                          'F_N_R_loop_counter_out',
                                                                                          'F_N_R_center_X',
                                                                                          'F_N_R_center_Y'])
     
        
    def execute(self, userdata ):       
                
        #rospy.sleep(10)
        rospy.loginfo('Executing state next unprocessed room.....')
        
        if userdata.F_N_R_loop_counter_in <= len(userdata.analyze_map_data_room_center_x_):
            find_next_unprocessed_room_action_server_result_ = find_next_unprocessed_room( userdata.next_unprocessed_room_data_img_,
                                                                                           userdata.analyze_map_data_room_center_x_,
                                                                                           userdata.analyze_map_data_room_center_y_,
                                                                                           userdata.analyze_map_data_map_resolution_,
                                                                                           userdata.analyze_map_data_map_origin_x_,
                                                                                           userdata.analyze_map_data_map_origin_y_ )
                    
            #rospy.sleep(10)
        
            userdata.next_unprocessed_room_number_out = find_next_unprocessed_room_action_server_result_.room_number
            
            rospy.loginfo('Current room No: %d'%userdata.F_N_R_loop_counter_in)  
            userdata.F_N_R_loop_counter_out = userdata.F_N_R_loop_counter_in + 1  
            userdata.F_N_R_center_X = find_next_unprocessed_room_action_server_result_.center_position_x
            userdata.F_N_R_center_Y = find_next_unprocessed_room_action_server_result_.center_position_y           
                                  
            return 'location'

        else:
            return 'no_rooms'

     
     
class GoToLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['successful','unsuccessful'],input_keys=['G_T_L_data_img',
                                                                                     'analyze_map_data_map_resolution_',
                                                                                     'analyze_map_data_map_origin_x_',
                                                                                     'analyze_map_data_map_origin_y_',
                                                                                     'F_N_R_center_X',
                                                                                     'F_N_R_center_Y',
                                                                                     'R_L_F_random_location_x',
                                                                                     'R_L_F_random_location_y',
                                                                                     'G_T_L_loop_counter_in'],
                                                                        output_keys=['G_T_L_loop_counter_out'])        
        
    def execute(self, userdata):
        
        #rospy.sleep(10)
        rospy.loginfo('Executing state Go_To_Location')        
        
        if userdata.G_T_L_loop_counter_in == 0 :
            G_T_L_data_result = to_goal(userdata.G_T_L_data_img,
                                        userdata.F_N_R_center_X , 
                                        userdata.F_N_R_center_Y,
                                        userdata.analyze_map_data_map_resolution_,
                                        userdata.analyze_map_data_map_origin_x_,
                                        userdata.analyze_map_data_map_origin_y_ )
            
        else:
             G_T_L_data_result = to_goal(userdata.G_T_L_data_img,
                                         userdata.R_L_F_random_location_x,
                                         userdata.R_L_F_random_location_y,
                                         userdata.analyze_map_data_map_resolution_,
                                         userdata.analyze_map_data_map_origin_x_,
                                         userdata.analyze_map_data_map_origin_y_)
                    
        if G_T_L_data_result.resultant == 'True':        
            return 'successful'
        
        else:
            userdata.G_T_L_loop_counter_out = userdata.G_T_L_loop_counter_in + 1
            return 'unsuccessful' 
    
    
    
class Random_Location_Finder(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['re_locate','unsuccessful_five_times'],input_keys=['R_L_F_data_img_in',
                                                                                               'R_L_F_room_number',
                                                                                               'analyze_map_data_room_min_x_',
                                                                                               'analyze_map_data_room_max_x_',
                                                                                               'analyze_map_data_room_min_y_',
                                                                                               'analyze_map_data_room_max_y_',
                                                                                               'R_L_F_counter_in'],
                                                                                  output_keys=['R_L_F_random_location_x',
                                                                                               'R_L_F_random_location_y',
                                                                                               'R_L_F_counter_out',
                                                                                               'R_L_F_data_img_out'])        
        
    def execute(self, userdata):
        
        #rospy.sleep(10)
        rospy.loginfo('Executing state Go_To_Location')        
        
        R_L_F_data_result = random_Location(userdata.R_L_F_data_img_in,
                                            userdata.R_L_F_room_number,
                                            userdata.analyze_map_data_room_min_x_,
                                            userdata.analyze_map_data_room_max_x_,
                                            userdata.analyze_map_data_room_min_y_,
                                            userdata.analyze_map_data_room_max_y_,
                                            userdata.R_L_F_counter_in)
        
        userdata.R_L_F_random_location_x = R_L_F_data_result.random_location_x
        userdata.R_L_F_random_location_y = R_L_F_data_result.random_location_y
        
        if userdata.R_L_F_counter_in < 6:
            return 're_locate'
        
        else:
            print "unsuccessful rate: ",userdata.R_L_F_counter_in
            userdata.R_L_F_counter_out = 0 ;
            userdata.R_L_F_data_img_out = R_L_F_data_result.output_img
            return 'unsuccessful_five_times'
     


class InspectRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'],input_keys=['I_R_data_img_in',
                                                                     'I_R_room_number',
                                                                     'analyze_map_data_room_center_x_',
                                                                     'analyze_map_data_room_center_y_',
                                                                     'analyze_map_data_room_min_x_',
                                                                     'analyze_map_data_room_max_x_',
                                                                     'analyze_map_data_room_min_y_',
                                                                     'analyze_map_data_room_max_y_',
                                                                     'analyze_map_data_map_resolution_',
                                                                     'analyze_map_data_map_origin_x_',
                                                                     'analyze_map_data_map_origin_y_'],
                                                        output_keys=['I_R_data_img_out'])         
        
    def execute(self, userdata):
        
        #rospy.sleep(10)
        rospy.loginfo('Executing state Inspect_Room')        

        I_R_data_result=inspect_Room( userdata.I_R_data_img_in,
                                    userdata.I_R_room_number,
                                    userdata.analyze_map_data_room_center_x_,
                                    userdata.analyze_map_data_room_center_y_,
                                    userdata.analyze_map_data_room_min_x_,
                                    userdata.analyze_map_data_room_max_x_,
                                    userdata.analyze_map_data_room_min_y_,
                                    userdata.analyze_map_data_room_max_y_,
                                    userdata.analyze_map_data_map_resolution_,
                                    userdata.analyze_map_data_map_origin_x_,
                                    userdata.analyze_map_data_map_origin_y_)
        #rospy.sleep(10)
        
        userdata.I_R_data_img_out = I_R_data_result.output_img
        
        return 'finished'          



def main():
    
    rospy.init_node('smach_exploration')

    sm_top = smach.StateMachine(outcomes=['finish'])    

    with sm_top:
        smach.StateMachine.add('ANALYZE_MAP', AnalyzeMap(),transitions={'list_of_rooms':'UNPROCESSED_ROOM'},
                   remapping={'analyze_map_data_img_':'sm_img'})
               
        sm_sub = smach.StateMachine(outcomes=['arrived','no_more_rooms_left'],input_keys=['sm_img',
                                                                                        'analyze_map_data_map_resolution_',
                                                                                        'analyze_map_data_map_origin_x_',
                                                                                        'analyze_map_data_map_origin_y_',
                                                                                        'analyze_map_data_room_center_x_', 
                                                                                        'analyze_map_data_room_center_y_',
                                                                                        'analyze_map_data_room_min_x_',
                                                                                        'analyze_map_data_room_max_x_',
                                                                                        'analyze_map_data_room_min_y_',
                                                                                        'analyze_map_data_room_max_y_'],
                                                                           output_keys=['sm_RoomNo'])
        sm_sub.userdata.sm_counter = 1
        sm_sub.userdata.sm_location_counter = 0
        
        with sm_sub:                 
            smach.StateMachine.add('FIND_NEXT_ROOM', NextUnprocessedRoom(),transitions={'location':'GO_TO_LOCATION','no_rooms':'no_more_rooms_left'},
                   remapping={'next_unprocessed_room_data_img_':'sm_img',
                              'F_N_R_room_number_in':'sm_RoomNo',
                              'next_unprocessed_room_number_out':'sm_RoomNo',
                              'F_N_R_loop_counter_in':'sm_counter',
                              'F_N_R_loop_counter_out':'sm_counter'})
            
            smach.StateMachine.add('GO_TO_LOCATION', GoToLocation(),transitions={'successful':'arrived','unsuccessful':'RANDOM_LOCATION_FINDER'},
                   remapping={'G_T_L_data_img':'sm_img',
                              'G_T_L_loop_counter_in':'sm_location_counter',
                              'G_T_L_loop_counter_out':'sm_location_counter'})  
            
            smach.StateMachine.add('RANDOM_LOCATION_FINDER', Random_Location_Finder(),transitions={'re_locate':'GO_TO_LOCATION','unsuccessful_five_times':'FIND_NEXT_ROOM'},
                   remapping={'R_L_F_data_img_in':'sm_img',
                              'R_L_F_data_img_out':'sm_img',
                              'R_L_F_counter_in':'sm_location_counter',
                              'R_L_F_counter_out':'sm_location_counter',
                              'R_L_F_room_number':'sm_RoomNo'}) 
        
        smach.StateMachine.add('UNPROCESSED_ROOM', sm_sub ,transitions={'arrived':'INSPECT_ROOM','no_more_rooms_left':'finish'})
      
        
        smach.StateMachine.add('INSPECT_ROOM', InspectRoom(),transitions={'finished':'UNPROCESSED_ROOM'},
                   remapping={'I_R_data_img_in':'sm_img',
                              'I_R_data_img_out':'sm_img',
                              'I_R_room_number':'sm_RoomNo'})
        
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()
    
    #rospy.spin()
    
    sis.stop()


if __name__ == '__main__':
    main()
    
    
    
    
    