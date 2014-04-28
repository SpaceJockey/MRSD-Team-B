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
from spacejockey.srv import AddWaypoints
from spacejockey.msg import Waypoint
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import tf

window = spacejockey.config("/gui")
frame_names = rospy.get_param('/planner/frame_names')


def MtoPx(x, y):
	"""convert an x,y point to screen coordinates"""
	n_x = (x  / window.scale) + window.origin.x
	n_y = window.origin.y - (y / window.scale)
	return (int(n_x), int(n_y))


def PxtoM(x, y):
	"""convert an x,y point from screen coordinates"""
	n_x = (x - window.origin.x) * window.scale
	n_y = (window.origin.y - y) * window.scale
	return (n_x, n_y)

class CV_App(object):
	def __init__(self):
		self.bridge = CvBridge() #used for getting ROS image topics as CV images

		self.battSize = (64, 8) #size of the battery indicator
		self.batt = 0.0
		self.markers = {}
		self.rFrames = {} #robot foot locations

		#used by click-drag callback
		self._startPt = None
		self._currPt = (0, 0)
		rospy.wait_for_service('add_waypoints') 
		try:
			self.add_waypoints = rospy.ServiceProxy('add_waypoints', AddWaypoints)
		except rospy.ServiceException, e:
			rospy.logerr("Service setup failed: " + str(e))

		self.tfList = tf.TransformListener()

		#initialize window
		cv2.namedWindow(window.title) # Create a OpenCV highgui window	
		cv.SetMouseCallback(window.title, self.click_cb)

		self.clean = cv2.imread(rospy.get_param('/clean_env_map'))
		self.shape = self.clean.shape
		self.dirty = np.zeros(self.shape, np.uint8)
		self.redraw()

	def __del__(self):
		cv2.destroyAllWindows() # Close everything.

	def redraw(self):
		#TODO: check for closed window and delete app...

		#this is our output image
		canvas = cv2.addWeighted(self.clean, .25, self.dirty, .75, 0)

		#draw click-drag bounding box
		if self._startPt != None:
			cv2.rectangle(canvas, self._startPt, self._currPt, cv.CV_RGB(0, 255, 0))

		#draw battery status indicator
		g = 255 if self.batt > .25 else 0 #yellow or green > 25%
		r = 255 if self.batt < .50 else 0 #yellow or red < 50%
		c = cv.CV_RGB(r, g, 0) #indicator color

		w = int(self.batt * self.battSize[0]) #indicator width
		if w < 0:
			w = 0
		cv2.rectangle(canvas, (10 , 10), (14 + self.battSize[0], 14 + self.battSize[1]), c)
		cv2.rectangle(canvas, (14 + self.battSize[0], 12 + (self.battSize[1] / 4)), (16 + self.battSize[0], 12 + ((3 * self.battSize[1])/4)), c, thickness = -1)
		cv2.rectangle(canvas, (12 , 12), (w + 12, 12 + self.battSize[1]), c, thickness = -1)

		#draw robot position
		c = cv.CV_RGB(0, 0, 196) #robot color
		for node in frame_names.values():
			try:
				(loc, rot) = self.tfList.lookupTransform('world', node, rospy.Time(0))
				self.rFrames[node] = MtoPx(loc[0], loc[1])
			except Exception as e:
				continue
		for foot in self.rFrames.values():
			cv2.circle(canvas, foot, 4, c, thickness = -1)

		#draw/delete viz markers
		self.clean_markers()
		for marker in self.markers.values():
			self.draw_marker(canvas, marker)

		cv2.imshow(window.title, canvas)
		cv2.waitKey(1)

	#delete expired markers...
	def clean_markers(self):
		now = rospy.Time.now()
		for key in self.markers.keys():
			marker = self.markers[key]
			if (marker.lifetime != rospy.Duration(0) and (marker.header.stamp + marker.lifetime) < now):
				rospy.loginfo('Deleting expired marker: ' + key)
				del self.markers[key] #delete expired marker

	#currently, only CUBE and SPHERE are implemented
	#returns a boolean on whether or not to delete the marker
	def draw_marker(self, image, marker):
		r = int(marker.color.r * 255)
		g = int(marker.color.g * 255)
		b = int(marker.color.b * 255)
		c = cv.CV_RGB(r, g, b)

		x_size = int(marker.scale.x / window.scale) / 2
		y_size = int(marker.scale.y / window.scale) / 2
		center = MtoPx(marker.pose.position.x, marker.pose.position.y)
		if marker.type == Marker.CUBE:
			x, y = center
			cv2.rectangle(image, (x-x_size , y-y_size), (x+x_size, y+y_size), c, thickness = 2) #probably a surface flaw
		else:
			cv2.circle(image, center, x_size, c)

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
		mX, mY = PxtoM(x, y)
		if event == CV_App.MOVE:
			self._currPt = (x, y) #PX
			return
		elif event == CV_App.RIGHT_RELEASE: #add a single move waypoint
			waypoints = [Waypoint(Waypoint.MOVE, mX, mY)]
		elif event == CV_App.LEFT_CLICK: #store start point for click-drags
			self._startPt = (x, y) #PX
		elif event == CV_App.LEFT_RELEASE: #store start point for click-drags
			#TODO: add click-drag support
			waypoints = [Waypoint(Waypoint.VIEW, mX, mY)]
			self._startPt = None
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
		if msg.header.frame_id == 'world': #only show global markers, no AR tags...
			if msg.action == Marker.DELETE:
				try:
					del self.markers[name]
					rospy.loginfo('Deleting marker: ' + name)
				except KeyError:
					pass
			else:
				self.markers[name] = msg	
		else:
			rospy.loginfo('Ignoring non-world frame marker: ' + name)

if __name__ == '__main__':
	#set up window
	rospy.init_node('sj_gui')
	app = CV_App()

	#Setup ROS Callbacks...
	rospy.Subscriber("/dirty_map", Image, app.img_cb) 				  #inspection image
	rospy.Subscriber('/battery_state', Float32, app.batt_cb)  		  #battery state
	rospy.Subscriber('/visualization_marker', Marker, app.marker_cb)  #flaw markers

	rospy.loginfo('GUI window Online')
	rate = rospy.Rate(30) #update at 30 frames a second
	while not rospy.is_shutdown():
		app.redraw()
		rate.sleep()

	#Clean up openCV windows
	del app