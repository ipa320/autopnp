

#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import actionlib
import autopnp_tool_change.msg 

class GoBackToStart:
	
	def go_back_to_start_client(self, goal_name, goal_state):
		go_back_to_start_client = actionlib.SimpleActionClient('go_back_to_start_action', autopnp_tool_change.msg.ToolChangeAction)
		go_back_to_start_client.wait_for_server()

		# Creates a goal to send to the action server.
		goal = autopnp_tool_change.msg.ToolChangeGoal()
		goal.goal = goal_name
		goal.state = goal_state

		# Sends the goal to the action server.
		go_back_to_start_client.send_goal(goal)

		# Waits for the server to finish performing the action.
		finished_before_timeout = go_back_to_start_client.wait_for_result(rospy.Duration(300, 0))

   		if finished_before_timeout:
   			result = go_back_to_start_client.get_result()
		   	state = go_back_to_start_client.get_state()
		   	
			print "Action go_back_to_start finished with state: %s"%state
			print "Action go_back_to_start finished with result:", result
			
		# Prints out the result of executing the action
   		return result # State after waiting for GoToStartPositionAction


