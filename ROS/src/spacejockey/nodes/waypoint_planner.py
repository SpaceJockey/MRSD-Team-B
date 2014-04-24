#!/usr/bin/env python
from Tkinter import *
from copy import deepcopy
import math
ferr = 0.0001 #floating point comparison error correction term

#ROS Imports
import rospy
import spacejockey
from spacejockey.msg import MajorPlanAction
from spacejockey.srv import *
from std_msgs.msg import Int16
import tf

config = spacejockey.config("/planner")
window = spacejockey.config("/gui")
frame_names = rospy.get_param('/planner/frame_names')
node_names = ("front_foot", "center_foot", "rear_foot")

class Point:
	"""Point class stores and X,Y pair in meters, and provides scaling to/from the screen size, theta is optional"""
	PX = 1
	M = 2
	def __init__(self, x = 0, y = 0, units = M):
		global config
		if (units == Point.PX):
			self.x = (x - window.origin.x) * window.scale
			self.y = (window.origin.y - y) * window.scale
		else:
			self.x = x
			self.y = y

	#returns output values in screen coordinates
	def toScreen(self):
		global config
		n_x = (self.x  / window.scale) + window.origin.x
		n_y = window.origin.y - (self.y / window.scale)
		return(n_x, n_y)

	#returns the pythagorean distance between two points
	def distTo(self, other):
		return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

	#returns the global bearing of the other point
	def angleTo(self, other):
		return math.atan2(other.y - self.y, other.x - self.x) 

#returns the difference between two angles
def angleDiff(x, y): 
	return math.atan2(math.sin(x-y), math.cos(x-y))

class Waypoint(Point):
    VIEW = 1
    MOVE = 2

    def __init__(self, x = 0, y = 0, action = MOVE, units = Point.M):
		Point.__init__(self, x,y,units)
		self.action = action
	
#main application class
class App:
	def __init__(self, master):
		global config
		self.root = master
		frame = Frame(master)
		frame.pack()

		self.srv = rospy.Service('major_planner', MajorPlanner, self.handleMajorRequest)
		self.current_major_id = 1

		self.tfList = tf.TransformListener()
		rospy.Subscriber('/battery_state', Int16, self.handleBatt)

		self.canvas = Canvas(frame, bg="black", height = window.height, width = window.width, confine = "true", cursor = "cross")
		self.canvas.pack(side=TOP)
		self.canvas.bind("<Button-1>", self.clickCB) 
		self.canvas.bind("<Button-3>", self.clickCB) 
		
		self.status = StringVar()
		self.status.set("IDLE")
		self.statusbox = Label(frame, textvariable = self.status, relief=SUNKEN, width = 20);
		self.statusbox.pack(side=RIGHT)

		self.batt = StringVar()
		self.batt.set("Battery: 100%")
		self.battbox = Label(frame, textvariable = self.batt, relief=SUNKEN, width = 20);
		self.battbox.pack(side=LEFT)

		#robot configuration locations are represented as tuples of AngledPoints representing each foot.
		self.confignames = ("front_foot", "center_foot", "rear_foot")
		self.currconfig = [Point(config.extend.min),Point(),Point(-config.extend.min)]
		self.planconfig = [Point(config.extend.min),Point(),Point(-config.extend.min)]
		self.currMove = None

		#set up queues
		self.waypoints = []
		self.update()

	def handleBatt(self, msg):
		self.batt.set("Battery: " + str(msg.data) + "%")

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

	def drawPoint(self, p, color = "blue", tag = ""):
		x,y = p.toScreen()
		self.canvas.create_oval(x-3,y-3,x+3,y+3, outline=color, tags=tag)
	
	def redraw(self):
		#update status indicator
		if(len(self.waypoints) == 0):
			self.status.set("Idle")
		else:
			self.status.set("Moving")

		#redraw everything
		self.canvas.delete(ALL) 

		#redraw self.waypoints
		for w in self.waypoints:
			color = "green" if w.action == Waypoint.MOVE else "red"
			self.drawPoint(w,color,"waypoint")		

		#draw Robot config
		for f in self.currconfig:
			self.drawPoint(f,"blue","foot")

		#camera view effect deprecated since Songjie's code will be displaying here


	def update(self):
		#check for ROS shutdown
		if rospy.is_shutdown():
			self.root.destroy()
			return

		#update robot draw state
		for node in frame_names.keys():
			try:
				(loc, rot) = self.tfList.lookupTransform('world', frame_names[node], rospy.Time(0))
				idx = self.confignames.index(node)
				self.currconfig[idx] = Point(loc[0], loc[1])
			except Exception as e:
				continue

		self.redraw()
		self.root.after(50, self.update)
			
	#on mouse clicks, add a waypoint
	def clickCB(self, event):
		canvas = event.widget
		x = canvas.canvasx(event.x)
		y = canvas.canvasy(event.y)
		action = Waypoint.MOVE if event.num == 1 else Waypoint.VIEW
		wp = Waypoint(x, y, action, Point.PX)
		self.waypoints.append(wp)
		rospy.loginfo("Added waypoint at: " + str((wp.x, wp.y)))
		self.redraw()

#set up window
root = Tk()   
root.wm_title("Planner Prototype")
root.config(background="black")

#start up ROS
rospy.init_node('waypoint_planner')
rospy.loginfo('Waypoint Planner Online')
#rospy.sleep(0.5)
app = App(root)

#start the GUI
root.mainloop()