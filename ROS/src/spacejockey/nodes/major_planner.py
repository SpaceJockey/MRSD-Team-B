#!/usr/bin/env python
import math
ferr = 0.01 #floating point comparison error correction term, 1 cm accuracy

#ROS Imports
import rospy
import spacejockey
from spacejockey.geometry import *
from spacejockey.srv import *
from spacejockey.msg import Waypoint
from visualization_msgs.msg import Marker
import tf

config = spacejockey.config("/planner")
frame_names = rospy.get_param('/planner/frame_names')

#main application class
class Planner:
	def __init__(self):
		self.current_major_id = 1
		self.outputSrv = rospy.Service('major_planner', MajorPlanner, self.handleMajorRequest)
		
		self.inputSrv = rospy.Service('add_waypoints', AddWaypoints, self.handleWaypointRequest)
		self.tfList = tf.TransformListener()

		#used for updating the GUI
		self.vizPub = rospy.Publisher('/visualization_marker', Marker)

		#set up queues
		self.rFrames = {}
		self.waypoints = []
		self.wp_ids = {}	#used for tracking vizualization instances
		self.curr_wp_id = 0

	def publishWaypointMarker(self, wp, delete = False):
		marker = Marker()
		marker.header.frame_id = "world"  
		marker.header.stamp = rospy.Time.now()
		marker.ns = "waypoints"
		marker.id = self.wp_ids[wp]
		marker.lifetime = rospy.Duration(0)
		if delete:
			marker.action = marker.DELETE
		else:
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.pose.position.x = wp.x
			marker.pose.position.y = wp.y
			marker.pose.orientation.w = 1.0
			marker.scale.y = 0.02
			marker.scale.x = 0.02
			marker.scale.z = 0.01
			marker.color.a = 1.0
			if wp.action == Waypoint.VIEW:
				marker.color.g = 1.0
			else:
				marker.color.r = 1.0

		# print marker
		self.vizPub.publish(marker)

	def removeWaypoint(self, wp):
		self.waypoints.remove(wp)
		self.publishWaypointMarker(wp, True)
		del self.wp_ids[wp]

	def handleWaypointRequest(self, req):
		#sort waypoints by range
		try:
			(loc, rot) = self.tfList.lookupTransform('world', 'robot', rospy.Time(0))
			robot = Point(loc[0], loc[1])
			req.waypoints.sort(key = lambda wp: robot.distTo(wp))
		except:
			pass

		for wp in req.waypoints:
			self.waypoints.append(wp)
			self.wp_ids[wp] = self.curr_wp_id
			self.curr_wp_id += 1
			if wp.action == Waypoint.VIEW:
				rospy.loginfo("Added view waypoint at: " + str((wp.x, wp.y)))
			else:
				rospy.loginfo("Added move waypoint at: " + str((wp.x, wp.y)))
			self.publishWaypointMarker(wp)
		return AddWaypointsResponse(True)

	def handleMajorRequest(self, req):
		global config, ferr
		#req should contain the last completed major_ID
		#TODO: check it!

		frontViewing = False
		for node in frame_names.keys():
			try:
				(loc, rot) = self.tfList.lookupTransform('world', frame_names[node], rospy.Time(0))
				self.rFrames[node] = Point(loc[0], loc[1])
				if node == 'front_foot' and loc[2] > ferr: #front foot is off the ground
					frontViewing = True
			except Exception as e:
				rospy.logwarn(e)
				continue

		#output pseudoparams
		tgtPoint = Point()
		node_name = "front_foot"
		action = MajorPlannerResponse.STEP
		sleep = rospy.Duration(0)

		if self.waypoints: #waypoints is not empty, plan a move
			wp = self.waypoints[0]
			#get target state
			tgtAngle = self.rFrames['center_foot'].angleTo(wp)
			tgtDist = self.rFrames['center_foot'].distTo(wp)
			
			#FIXME: this is way hacktackular!
			i = 0
			while wp.action == Waypoint.VIEW and tgtDist < config.view.min and i < 30: 
				#too close, reque waypoint for later...
				self.waypoints.remove(wp)
				self.waypoints.append(wp)
				wp = self.waypoints[0]
				tgtDist = self.rFrames['center_foot'].distTo(wp)
				i += 1 #iteration limit to prevent infinite loop...


			if wp.action == Waypoint.VIEW:
				tgtDist -= config.view.opt

			#more pseudoparams
			extend = config.extend.min  #desired extension
			newtheta = tgtAngle	#init desired angle

			#distances and angle error terms
			frontTheta = self.rFrames['center_foot'].angleTo(self.rFrames['front_foot'])
			backTheta = self.rFrames['rear_foot'].angleTo(self.rFrames['center_foot'])
			frontRel = angleDiff(tgtAngle, frontTheta) 							#relative angle between front and target
			backRel = angleDiff(tgtAngle, backTheta)  							#relative angle between back and target
			d1 = self.rFrames['center_foot'].distTo(self.rFrames['front_foot'])			#distance between front and middle
			d2 = self.rFrames['center_foot'].distTo(self.rFrames['rear_foot']) 			#distance between middle and rear

			#move decision tree
			if(abs(backRel) > config.angle.dead or abs(frontRel) > (config.angle.max - config.angle.dead)): 		#feet not pointing at the thing
				if(abs(frontRel) >= abs(backRel) or frontViewing): 		#rotate feet, starting with whichever is farther away
					newtheta = backTheta + math.copysign(min(abs(backRel), config.angle.max), backRel)
				else:
					newtheta = frontTheta + math.copysign(min(abs(frontRel), config.angle.max), frontRel)
					node_name = "rear_foot"
					extend = -config.extend.min
			else:										#move towards the thing
				if(wp.action == Waypoint.VIEW and tgtDist < (config.view.max - config.view.opt)):	#view plan action
					action = MajorPlannerResponse.VIEW
					sleep = rospy.Duration(3)
					extend = config.view.extend
					self.removeWaypoint(wp)
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
						self.removeWaypoint(wp)

			#derive final joint coordinates
			tgtPoint.x = self.rFrames['center_foot'].x + (extend * math.cos(newtheta))
			tgtPoint.y = self.rFrames['center_foot'].y + (extend * math.sin(newtheta))
		else: #empty waypoint queue
			action = MajorPlannerResponse.SLEEP
			sleep = rospy.Duration(1)
			wp = Waypoint()

		x, y = (wp.x, wp.y)
		if action == MajorPlannerResponse.STEP:
			x, y = (tgtPoint.x, tgtPoint.y) 

		resp = MajorPlannerResponse(self.current_major_id, action, node_name, x,y, sleep)
		rospy.logdebug('Major Move: ' + str(resp))
		self.current_major_id += 1
		return resp

#TODO: add rate arguments, others useful args?
if __name__ == '__main__':
	rospy.init_node('major_planner')

	#start up planner
	planner = Planner()

	rospy.loginfo('Major Planner Online')
	rospy.spin()