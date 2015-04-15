#!/usr/bin/python

import roslib
roslib.load_manifest('cob_script_server')
import rospy
from std_msgs.msg import String
import threading 
from simple_script_server import simple_script_server
from cob_srvs.srv import Trigger

import sys, os
from PyQt4.QtCore import QThread, pyqtSignal
from PyQt4.QtGui import QApplication, QMainWindow
from Ui_MainWindow import Ui_MainWindow

app = QApplication(sys.argv)
window = QMainWindow()
ui = Ui_MainWindow()
ui.setupUi(window)

# ROS - Worker Thread
class RosThread(QThread):
    add = pyqtSignal(str)
    rem = pyqtSignal(str)

    name_gripper="3-Finger-Hand"
    name_vacuum ="Vacuum-Cleaner"

    def __init__(self, ui, parent = None):
        QThread.__init__(self, parent)
        self.ui = ui
        self.mode="linear"
        self.add[str].connect(self.onAdd)
        self.rem[str].connect(self.onRem)
        self.sub1 = rospy.Subscriber("/chromosom/addComponent", String, self.add_callback)
        self.sub2 = rospy.Subscriber("/chromosom/remComponent", String, self.rem_callback)
        self.pub_add = rospy.Publisher('/chromosom/addComponent', String)
        self.pub_rem = rospy.Publisher('/chromosom/remComponent', String)

        ui.btnOpen.clicked.connect(self.open_gripper)
        ui.btnClose.clicked.connect(self.close_gripper)

        ui.btnOn.clicked.connect(self.power_on)
        ui.btnOff.clicked.connect(self.power_off)

        ui.btnToolChange.clicked.connect(self.tool_change)

        ui.rLin.clicked.connect(self.lin)
        ui.rDWA.clicked.connect(self.DWA)
        ui.btnA.clicked.connect(self.A)
        ui.btnB.clicked.connect(self.B)

        self.sss = simple_script_server()
 
    def run(self):        
        rospy.spin()
 
    def add_callback(self, data):
        #print data.data
        self.add.emit(data.data)
    def rem_callback(self, data):
        print data.data
        self.rem.emit(data.data)

    def onAdd(self, data):
        if data==self.name_gripper:
           self.ui.gripper.show()
        if data==self.name_vacuum:
           self.ui.vacuum.show()
    def onRem(self, data):
        if data==self.name_gripper:
           self.ui.gripper.hide()
        if data==self.name_vacuum:
           self.ui.vacuum.hide()

    def open_gripper(self):
        self.sss.move("sdh", "cylopen")
    def close_gripper(self):
	self.sss.move("sdh", "home")

    def power_off(self):
	#os.system("rosservice call /vacuum_cleaner_controller/power_on")
	vacuum_recover_service_name = '/vacuum_cleaner_controller/recover'
	vacuum_on_service_name = '/vacuum_cleaner_controller/power_on'
	vacuum_off_service_name = '/vacuum_cleaner_controller/power_off'

	# recover vacuum cleaner (turning on is more reliably thereafter)
	rospy.wait_for_service(vacuum_recover_service_name) 
	try:
		req = rospy.ServiceProxy(vacuum_recover_service_name, Trigger)
		resp = req()
	except rospy.ServiceException, e:
		print "Service call to vacuum recover failed: %s"%e

	# turn vacuum cleaner off
	rospy.wait_for_service(vacuum_off_service_name) 
	try:
		req = rospy.ServiceProxy(vacuum_off_service_name, Trigger)
		resp = req()
	except rospy.ServiceException, e:
		print "Service call to vacuum off failed: %s"%e
    def power_on(self):
	#os.system("rosservice call /vacuum_cleaner_controller/power_on")
	vacuum_recover_service_name = '/vacuum_cleaner_controller/recover'
	vacuum_on_service_name = '/vacuum_cleaner_controller/power_on'
	vacuum_off_service_name = '/vacuum_cleaner_controller/power_off'

	# recover vacuum cleaner (turning on is more reliably thereafter)
	rospy.wait_for_service(vacuum_recover_service_name) 
	try:
		req = rospy.ServiceProxy(vacuum_recover_service_name, Trigger)
		resp = req()
	except rospy.ServiceException, e:
		print "Service call to vacuum recover failed: %s"%e

	# turn vacuum cleaner on
	rospy.wait_for_service(vacuum_on_service_name) 
	try:
		req = rospy.ServiceProxy(vacuum_on_service_name, Trigger)
		resp = req()
	except rospy.ServiceException, e:
		print "Service call to vacuum on failed: %s"%e

    def tool_change(self):
	os.system("rosrun autopnp_scenario autopnp_automatica_toolchange.py &")

    def lin(self):
        self.mode="linear"
        self.pub_add.publish(String("Linear-Navigation"))
        self.pub_rem.publish(String("DWA"))

    def DWA(self):
        self.mode="omni"
        self.pub_rem.publish(String("Linear-Navigation"))
        self.pub_add.publish(String("DWA"))

    def A(self):
        self.sss.move("base", [-0.13,-0.6,0.85], mode=self.mode)
    def B(self):
        self.sss.move("base", [0.6,0.58,1.1], mode=self.mode)

ui.gripper.hide()
ui.vacuum.hide()

rospy.init_node('chromsome_gut')
th = RosThread(ui)
th.start()

window.show()
sys.exit(app.exec_())

th.join()
