#!/usr/bin/env python
from Tkinter import *
from copy import deepcopy
import math
ferr = 0.0001 #floating point comparison error correction term

#ROS Imports
import rospy
import spacejockey
from spacejockey.geometry import *
from spacejockey.srv import *
from spacejockey.msg import Waypoint
import tf

config = spacejockey.config("/planner")
frame_names = rospy.get_param('/planner/frame_names')

#main application class
class Planner:
	def __init__(self):

		self.outputSrv = rospy.Service('major_planner', MajorPlanner, self.handleMajorRequest)
		self.current_major_id = 1

		self.inputSrv = rospy.Service('add_waypoints', AddWaypoints, self.handleWaypointRequest)

		self.tfList = tf.TransformListener()

		#robot configuration locations are represented as tuples of AngledPoints representing each foot.
		self.confignames = ("front_foot", "center_foot", "rear_foot")
		self.currconfig = [Point(config.extend.min),Point(),Point(-config.extend.min)]

		#set up queues
		self.waypoints = []

	def publishWaypointMarkers(self): #FIXME!
		pass

	def handleWaypointRequest(self, req):
		#TODO: implement this...
		for wp in req.waypoints:
			self.waypoints.append(wp)
			rospy.loginfo("Added waypoint at: " + str((wp.x, wp.y)))
		return AddWaypointsResponse(True)

	def handleMajorRequest(self, req):
		global config, ferr
		#req should contain the last completed major_ID
		#TODO: check it!

		#output pseudoparams
		tgtPoint = Point()
		node_name = "front_foot"
		action = MajorPlannerResponse.STEP

		if self.waypoints: #waypoints is not empty, plan a move
			wp = self.waypoints[0]

			#get target state
			tgtAngle = self.currconfig[1].angleTo(wp)
			tgtDist = self.currconfig[1].distTo(wp) 
			if wp.action == Waypoint.VIEW: #stop short of view self.waypoints to get in ideal image range
				tgtDist -= config.view.opt

			#more pseudoparams
			extend = config.extend.min  #desired extension
			newtheta = tgtAngle	#init desired angle

			#distances and angle error terms
			frontTheta = self.currconfig[1].angleTo(self.currconfig[0])
			backTheta = self.currconfig[2].angleTo(self.currconfig[1])
			frontRel = angleDiff(tgtAngle, frontTheta) 					#relative angle between front and target
			backRel = angleDiff(tgtAngle, backTheta)  					#relative angle between back and target
			d1 = self.currconfig[1].distTo(self.currconfig[0])			#distance between front and middle
			d2 = self.currconfig[1].distTo(self.currconfig[2]) 			#distance between middle and rear

			#move decision tree
			if(abs(backRel) > ferr or abs(frontRel) > config.angle.max): 		#feet not pointing at the thing
				if(abs(frontRel) >= abs(backRel)): 		#rotate feet, starting with whichever is farther away
					newtheta = backTheta + math.copysign(min(abs(backRel), config.angle.max), backRel) #TODO
				else:
					newtheta = frontTheta + math.copysign(min(abs(frontRel), config.angle.max), frontRel) #TODO
					node_name = "rear_foot"
					extend = -config.extend.min
			else:										#move towards the thing
				if(wp.action == Waypoint.VIEW and tgtDist < config.view.max):	#view plan action
					action = MajorPlannerResponse.VIEW
					extend = config.view.extend
					self.waypoints.remove(wp) 			#is at target
				elif (d1 < (config.extend.max - ferr)): #extend front segment, accounting for error
					extend = config.extend.max
				elif (d2 > config.extend.min + ferr): 	#retract rear foot
					node_name = "rear_foot"
					extend = -config.extend.min
				else: 									#extend middle segment
					node_name = "center_foot"
					center_range = config.extend.max - config.extend.min
					extend = min(tgtDist, center_range)
					if (extend < center_range - ferr): 	#is at target
						self.waypoints.remove(wp)

			#derive final joint coordinates
			tgtPoint.x = self.currconfig[1].x + (extend * math.cos(newtheta))
			tgtPoint.y = self.currconfig[1].y + (extend * math.sin(newtheta))
		else: #empty waypoint queue
			action = MajorPlannerResponse.SLEEP
			wp = Waypoint()

		x, y = (wp.x, wp.y)
		if action == MajorPlannerResponse.STEP:
			x, y = (tgtPoint.x, tgtPoint.y) 

		resp = MajorPlannerResponse(self.current_major_id, action, node_name, x,y)
		self.current_major_id += 1
		return resp

if __name__ == '__main__':
	rospy.init_node('major_planner')

	#set up window
	planner = Planner()

	rospy.loginfo('Major Planner Online')
	rospy.spin()