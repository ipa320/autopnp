#!/usr/bin/python
import roslib
#roslib.load_manifest('autopnp_scenario')
import rospy
import tf
import os, sys, math
import imreg, numpy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

def say(sen):
	rospy.wait_for_service('/say')
	say = rospy.ServiceProxy('/say', String)
	try:
	  resp1 = say(sen)
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))

def send_base_pose(x, y, rot, var_trans, var_rot):
	# convert to pose message
	pwcs = PoseWithCovarianceStamped()
	pwcs.header.stamp = rospy.Time.now()
	pwcs.header.frame_id = "/map"

	pwcs.pose.pose.position.x = x
	pwcs.pose.pose.position.y = y
	pwcs.pose.pose.position.z = 0.0
	q = tf.transformations.quaternion_from_euler(0, 0, rot)
	pwcs.pose.pose.orientation.x = q[0]
	pwcs.pose.pose.orientation.y = q[1]
	pwcs.pose.pose.orientation.z = q[2]
	pwcs.pose.pose.orientation.w = q[3]
	
	pwcs.pose.covariance = \
[var_trans, 0, 0, 0, 0, 0, 
0, var_trans, 0 ,0 ,0 ,0,
0, 0, 0, 0, 0 ,0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, var_rot]

	print "publishing new robot pose"
	pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
	pub.publish(pwcs)

def OccupancyGrid2Im(m):
	#normalize
	d = []
	for i in xrange(len(m.data)):
		d.append( max(0,m.data[i])/100. )
	return numpy.asarray(d).reshape([m.info.width, m.info.height])

im_map=None
def cb_map(m):
	global im_map
	im_map = OccupancyGrid2Im(m)

def cb_local(m):
	global im_map
	if im_map==None: return

	#now both (map + local_map) are present
	im_local = OccupancyGrid2Im(m)
	say("Starting to localize myself")

	#registrate images
	match = imreg.find_match(im_map, im_local)
	print(match)
	if match==None:
		say("Ccould not localize. Abort.")
	say("Found a match with a score of "+str(match[3]))

	#get pose
	x = match[0]*m.info.resolution
	y = match[1]*m.info.resolution
	yaw = match[2]*math.pi/180.

	print("Pose: ",x,y,yaw)
	send_base_pose(x,y,yaw, 0.1, 0.1)

	#check for user
	(rx,ry,ryaw,rname) = rospy.get_param('relation', [0.,0.,0., "origin"])
	say("I am thinking that I am standing in relation to "+rname)
	dist = math.sqrt( (rx-x)*(rx-x) + (ry-y)*(ry-y) )
	angle = abs(yaw-ryaw)*180/math.pi
	say("Distance: "+str("%.1f" % dist))
	say("Delta angle: "+str(round(angle)))

if __name__ == '__main__':
	rospy.init_node('localize')
	rospy.Subscriber("/map", OccupancyGrid, cb_map)
	rospy.Subscriber("/map", OccupancyGrid, cb_local)
	rospy.spin()
	
