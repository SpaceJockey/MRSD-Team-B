#!/usr/bin/env python

#openCV imports
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os, sys
import numpy as np

#ROS Imports
import rospy
import spacejockey
from spacejockey.geometry import *
from spacejockey.srv import *
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
import tf

window = spacejockey.config("/gui")
frame_names = rospy.get_param('/planner/frame_names')

class CV_App(object):
	def __init__(self):
		self.bridge = CvBridge() #used for getting ROS image topics as CV images

		self.battery = 100
		self.markers = {}
		self.rframes = {} #robot foot locations

		self._startPt = (0, 0) #used by click-drag callback
		rospy.wait_for_service('add_waypoints') 
		try:
			self.add_waypoints = rospy.ServiceProxy('add_waypoints', AddWaypoints)
		except rospy.ServiceException, e:
			rospy.logerr("Service setup failed: " + str(e))

		self.tfList = tf.TransformListener()

		#initialize window
		cv2.namedWindow(window.title) # Create a OpenCV highgui window	
		cv.SetMouseCallback(window.title, self.click_cb)

		#TODO: perameterize config file name
		self.clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../config/clean_map.png")
		self.shape = self.clean.shape
		self.dirty = np.zeros(self.shape, np.uint8)
		self.redraw()

	def __del__(self):
		cv2.destroyAllWindows() # Close everything.

	def redraw(self):
		#TODO: check for closed window and delete app...

		#this is our output image
		canvas = cv2.addWeighted(self.clean, .25, self.dirty, .75, 0)

		#draw battery status indicator
		g = 255 if self.battery > 25 else 0 #yellow or green > 25%
		r = 255 if self.battery < 50 else 0 #yellow or red < 50%
		c = cv.CV_RGB(r, g, 0) #indicator color

		cv2.rectangle(canvas, (10 , 10), (114, 26), c)
		cv2.rectangle(canvas, (114 , 14), (116, 22), c, thickness = -1)
		cv2.rectangle(canvas, (12 , 12), (self.battery + 12, 24), c, thickness = -1)

		#draw robot position
		c = cv.CV_RGB(0, 0, 196) #robot color
		for node in frame_names.keys():
			try:
				(loc, rot) = self.tfList.lookupTransform('world', frame_names[node], rospy.Time(0))
				self.rframes[node] = Point(loc[0], loc[1])
			except Exception as e:
				continue
		for foot in self.rframes.values():
			cv2.circle(canvas, foot.toScreen(), 4, c, thickness = -1)

		cv2.imshow(window.title, canvas)
		cv2.waitKey(1)

	#Click callback static values
	MOVE = 0
	LEFT_BUTTON = 1
	RIGHT_BUTTON = 2

	LEFT_CLICK = 1
	LEFT_RELEASE = 4
	RIGHT_CLICK = 2
	RIGHT_RELEASE = 5

	def click_cb(self, event, x, y, button, args):
		waypoints = []
		if event == CV_App.MOVE:
			return
		elif event == CV_App.RIGHT_RELEASE: #add a single view waypoint
			waypoints = [Waypoint(x, y, Waypoint.VIEW, Point.PX)]
		elif event == CV_App.LEFT_CLICK: #store start point for click-drags
			self._startPt = Point(x, y)
		elif event == CV_App.LEFT_RELEASE: #store start point for click-drags
			#TODO: add click-drag support
			waypoints = [Waypoint(self._startPt.x, self._startPt.y, Waypoint.VIEW, Point.PX)]
		self.add_waypoints(waypoints) #run waypoint service callback
			
	def img_cb(self, msg):
		try:
			dirty = self.bridge.imgmsg_to_cv2(msg, "passthrough")
			assert dirty.shape == self.shape, 'Image size mismatch:' + str(dirty.shape) + " != " + str(self.shape)
		except Exception as e:
			rospy.logerr(str(e))
			return
		self.dirty = dirty

	def batt_cb(self, msg):
		self.batt = msg.data

	#vizualization_msgs callback
	#vizualization_messages are used for drawing defects and waypoints...
	def marker_cb(self, msg):
		name = msg.ns + "_" + str(msg.id)
		self.markers[name] = msg

if __name__ == '__main__':
	#set up window
	rospy.init_node('sj_gui')
	app = CV_App()

	#Setup ROS Callbacks...
	rospy.Subscriber("/dirty_map", Image, app.img_cb) 				  #inspection image
	rospy.Subscriber('/battery_state', Int16, app.batt_cb)  		  #battery state
	rospy.Subscriber('/visualization_marker', Marker, app.marker_cb)  #flaw markers

	rospy.loginfo('GUI window Online')
	rate = rospy.Rate(30) #update at 30 frames a second
	while not rospy.is_shutdown():
		app.redraw()
		rate.sleep()

	#Clean up openCV windows
	del app