#!/usr/bin/python
import roslib
roslib.load_manifest('autopnp_scenario')
import rospy
import tf
import os, sys, math
import imreg, numpy

from nav_msgs.msg import OccupancyGrid, GridCells
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from cob_sound.srv import SayText, SayTextRequest
from simple_script_server import simple_script_server

from exploration_detection_cleaning import currentRobotPose

def say(sen):
	print(sen)
	rospy.wait_for_service('/say')
	say_ = rospy.ServiceProxy('/say', SayText)
	s = SayTextRequest()
	s.text = sen
	try:
	  resp1 = say_(s)
	except rospy.ServiceException as exc:
	  print("Service did not process request: " + str(exc))
	return False

def global_localization():
	rospy.wait_for_service('/global_localization')
	s = rospy.ServiceProxy('/global_localization', Empty)
	try:
	  resp1 = s()
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

	global pose_pub
	pose_pub.publish(pwcs)

def OccupancyGrid2Im(m):
	#normalize
	d = []
	for i in xrange(len(m.data)):
		v=0.
		if m.data[i]>90 or m.data[i]<-10: v=1.
		d.append( v )
	return numpy.swapaxes(numpy.asarray(d).reshape([m.info.height, m.info.width]),0,1)

im_map=None
confirm=False

def cb_joy(msg):
	global confirm
	# check for joystick [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0] --> buttons 6+8+2 (= both right triggers + lower of the four buttons)
	if msg.buttons[0]==0 and msg.buttons[1]==1 and msg.buttons[2]==0 and msg.buttons[3]==0 and msg.buttons[4]==0 and msg.buttons[5]==1 and msg.buttons[6]==0 and msg.buttons[7]==1 and msg.buttons[8]==0 and msg.buttons[9]==0 and msg.buttons[10]==0 and msg.buttons[11]==0:
		confirm = True

done = 0

def init():
	global confirm
	sss = simple_script_server()
	say("I am ready. I will initialize myself.")
	say("Please confirm.")
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

	say("All components are ready. Please drive me around until I am localized.")

	return True

def cb_map(m):
	global done
	if done>0: return

	if init()!=True: return

	global_localization()
	done = 1

def cb_particles(msg):
	global done
	if done!=1: return
	arr_x=[]
	arr_y=[]
	arr_yaw=[]
	for p in msg.poses:
		quaternion = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		arr_x.append(p.position.x)
		arr_y.append(p.position.y)
		arr_yaw.append(euler[2])

	m = numpy.mean(numpy.asarray(arr_yaw), axis=0)
	for i in xrange(len(arr_yaw)):
		d = abs(arr_yaw[i]-m)
		if d>math.pi: arr_yaw[i]=d-math.pi

	dx = numpy.std(numpy.asarray(arr_x), axis=0)
	dy = numpy.std(numpy.asarray(arr_y), axis=0)
	dyaw=numpy.std(numpy.asarray(arr_yaw), axis=0)

	print(dx,dy,dyaw)

	if dx<0.2 and dy<0.2 and dyaw<0.2:
		say("I am now localized. Checking.")
		done = 2

		listener = tf.TransformListener()
		(trans,quat,rot) = currentRobotPose()

		print (trans, rot)
		x = trans[0]
		y = trans[1]
		yaw = rot[0]
		
		#check for user
		(rx,ry,ryaw,rname) = rospy.get_param('relation', [0.,0.,0., "origin"])
		say("I am thinking that I am standing in relation to "+rname)
		dist = math.sqrt( (rx-x)*(rx-x) + (ry-y)*(ry-y) )
		angle = abs(yaw-ryaw)*180/math.pi
		say("Distance: "+str("%.1f" % dist)+" meters")
		say("Delta angle: "+str(int(round(angle)))+" degrees")

		say("Please confirm within 10 seconds.")
		confirm = False
		for i in xrange(100):
			rospy.sleep(0.1)
			if confirm==True: break
		if confirm==False:
			say("Localization failed. Retrying.")
			global_localization()
			return

		say("Localized.")
		done = 3
		exit(0)

if __name__ == '__main__':
	rospy.init_node('localize')
	pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
	rospy.Subscriber("/map", OccupancyGrid, cb_map)
	rospy.Subscriber('/particlecloud', PoseArray, cb_particles)
	#rospy.Subscriber("/local_costmap_node/costmap/obstacles", GridCells, cb_local)
	rospy.Subscriber("/joy", Joy, cb_joy)
	rospy.spin()
	
