#!/usr/bin/python
import roslib
roslib.load_manifest('autopnp_scenario')
import rospy
from simple_script_server import simple_script_server
from localize import say

rospy.init_node('recover')

sss = simple_script_server()

say("recovering")

# recover components
components = ["head", "torso", "arm", "base"]
for c in components:
	handle_head = sss.recover(c)
	if handle_head.get_error_code() != 0:
		say("failed to recover "+c)

say("done")
