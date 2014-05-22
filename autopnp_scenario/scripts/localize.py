#!/usr/bin/python
import roslib
#roslib.load_manifest('autopnp_scenario')
import rospy
import tf
import os, sys, math
import imreg, numpy

from nav_msgs.msg import OccupancyGrid, GridCells
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Joy
from simple_script_server import simple_script_server

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
		v=0.
		if m.data[i]>50 or m.data[i]<0: v=1.
		d.append( v )
	return numpy.swapaxes(numpy.asarray(d).reshape([m.info.height, m.info.width]),0,1)

im_map=None
confirm=False

def cb_joy(msg):
	global confirm
	# check for joystick [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0] --> buttons 6+8+2 (= both right triggers + lower of the four buttons)
	if msg.buttons[0]==0 and msg.buttons[1]==1 and msg.buttons[2]==0 and msg.buttons[3]==0 and msg.buttons[4]==0 and msg.buttons[5]==1 and msg.buttons[6]==0 and msg.buttons[7]==1 and msg.buttons[8]==0 and msg.buttons[9]==0 and msg.buttons[10]==0 and msg.buttons[11]==0:
		confirm = True

def cb_map(m):
	global im_map
	global confirm
	im_map = OccupancyGrid2Im(m)
	imreg.imshow(im_map)

	return
	#now seems up, so turn around
	sss = simple_script_server()
	say("I am ready. I will initialize myself. Then I will turn around.")
	say("Please onfirm")
	confirm = False
	while confirm==False: rospy.sleep(0.1)

	# initialize components
	handle_head = sss.init("head")

	handle_torso = sss.init("torso")
	if handle_torso.get_error_code() != 0:
		return say("failed to initialize torso")
		
	handle_tray = sss.init("tray")

	handle_arm = sss.init("arm")
	if handle_arm.get_error_code() != 0:
		return say("failed to initialize arm")

	handle_sdh = sss.init("sdh")

	handle_base = sss.init("base")
	if handle_base.get_error_code() != 0:
		return say("failed to initialize base")
	
	# recover components
	handle_head = sss.recover("head")
	if handle_head.get_error_code() != 0:
		return say("failed to recover head")		

	#turn around
	for x in xrange(8): sss.move_base_rel('base', [0,0,math.pi/8])

	#ready
	im_map = tmp

def cb_local(m):
	global im_map
	if im_map==None: return

	#now both (map + local_map) are present
	mia=[0,0,0,0]
	for c in m.cells:
		x = int(c.x/cell_width)
		y = int(c.y/cell_width)
		mia = [min(x,mia[0]), min(y,mia[1]), max(x,mia[2]), max(y,mia[3]) ]
	d=[]
	for x in xrange(mia[2]-mia[0]):
		d.append([])
		for y in xrange(mia[3]-mia[1]):
			d[len(d)-1].append(0.)
	for c in m.cells:
		x = int(c.x/cell_width)
		y = int(c.y/cell_width)
		d[x][y]=1.
	im_local = numpy.asarray(d)
	say("Starting to localize myself")

	imreg.imshow(im_map, im_local, 0.125)

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
	rospy.Subscriber("/obstacles", GridCells, cb_local)
	rospy.Subscriber("/joy", Joy, cb_joy)
	rospy.spin()
	
