#!/usr/bin/env python
from Tkinter import *
from copy import deepcopy
import math
ferr = 0.0001 #floating point comparison error correction term
merr = 0.01 #move positioning accuracy
#import IPython

#ROS Imports
import roslib
#roslib.load_manifest('spacejockey')
import rospy
import sys
import spacejockey
from spacejockey.msg import MajorPlanAction
from std_msgs.msg import Int16
import tf

config = spacejockey.config("/planner")
window = spacejockey.config("/gui")
frame_names = rospy.get_param('/planner/frame_names')

#ROS publisher
actionPub = rospy.Publisher('major_actions', MajorPlanAction)

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

#class AngledPoint(Point):
#	def __init__(self, x = 0, y = 0, theta = 0, units = Point.M):
#		Point.__init__(self, x, y, units)
#		self.theta = theta
#
#	def __repr__(self):
#		return '[' + "{:.3f}".format(self.x) + ', ' + "{:.3f}".format(self.y) + ', ' + "{:.4f}".format(self.theta) + ']'

class Waypoint(Point):
    VIEW = 1
    MOVE = 2

    def __init__(self, x = 0, y = 0, action = MOVE, units = Point.M):
		Point.__init__(self, x,y,units)
		self.action = action

class MajorMove():
	STEP = MajorPlanAction.STEP
	VIEW = MajorPlanAction.VIEW

	FRONT = 0
	MIDDLE = 1
	REAR = 2

	NAMES = ("front_foot", "center_foot", "rear_foot")

	current_id = 0

	def __init__(self, point, node_id, waypoint, action = STEP):
		self.major_id = self.__class__.current_id
		self.__class__.current_id += 1
		self.point = point
		self.node_id = node_id
		self.action = action
		self.waypoint = waypoint

	def publish(self):
		global actionPub
		node_name = self.__class__.NAMES[self.node_id] if (self.action == self.__class__.STEP) else "camera"
		x, y = (self.point.x, self.point.y) if (self.action == self.__class__.STEP) else (self.waypoint.x, self.waypoint.y)
		return actionPub.publish(MajorPlanAction(self.major_id, self.action, node_name, x, y, 0.0))

	def isAtTarget(self):
		global ferr
		return (self.action == self.__class__.VIEW) or (self.node_id == self.__class__.MIDDLE and (self.point.distTo(self.waypoint) < ferr))

	def applyTo(self, rconfig):
		rconfig[self.node_id] = self.point
		#rconfig[1].theta = rconfig[2].theta

	
#main application class
class App:
	def __init__(self, master):
		global config
		self.root = master
		frame = Frame(master)
		frame.pack()

		self.tfList = tf.TransformListener()
		rospy.Subscriber('/battery_state', Int16, self.handleBatt)
		
		self.prevX = 0
		self.prevY = 0

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
		#self.setCurrMove(MajorMove(Point(.5, 0), 0, Point(.5, 0), MajorMove.VIEW)) #initial localization move

		#set up queues
		self.waypoints = []
		self.moves = []
		self.update()

	def handleBatt(self, msg):
		self.batt.set("Battery: " + str(msg.data) + "%")

	def setCurrMove(self, move):
		self.currMove = move
		move.publish()

	def drawPoint(self, p, color = "blue", tag = ""):
		x,y = p.toScreen()
		self.canvas.create_oval(x-3,y-3,x+3,y+3, outline=color, tags=tag)
		#if(isinstance(p, AngledPoint)):
		#	cTh = 6 * math.cos(p.theta)
		#	sTh = 6 * math.sin(p.theta)
		#	self.canvas.create_line(x - cTh, y + sTh, x + cTh, y - sTh, fill=color, tags=tag)
	
	def redraw(self):
		#update status indicator
		if(len(self.waypoints) == 0):
			self.status.set("Idle")
		else:
			self.status.set("Moving")

		#redraw everything
		self.canvas.delete(ALL) 

		#draw planned self.moves, useful for debugging
		#for m in self.moves:
		#	self.drawPoint(m.point,"navy","plan")

		#redraw self.waypoints
		for w in self.waypoints:
			color = "green" if w.action == Waypoint.MOVE else "red"
			self.drawPoint(w,color,"waypoint")		

		#draw Robot config
		for f in self.currconfig:
			self.drawPoint(f,"blue","foot")

		#draw camera view effect
		if len(self.moves) > 0 and (self.moves[0].action == MajorMove.VIEW):
			x,y = self.moves[0].waypoint.toScreen()
			fx, fy = self.moves[0].point.toScreen()
			theta = self.currconfig[1].angleTo(self.currconfig[0]) #self.moves[0].point.theta
			r = config.view.radius / window.scale
			cTh = r * math.cos(theta)
			sTh = r * math.sin(theta)
			self.canvas.create_oval(x-r, y-r, x+r, y+r, outline="red", tags="view")
			self.canvas.create_line(fx, fy, x - sTh, y - cTh, fill="red", tags="view")
			self.canvas.create_line(fx, fy, x + sTh, y + cTh, fill="red", tags="view")


	def update(self):
		#check for ROS shutdown
		if rospy.is_shutdown():
			self.root.destroy()
			return

		#update robot draw state
		now = rospy.Time.now()
		for node in frame_names.keys():
			try:
				self.tfList.waitForTransform('world', frame_names[node], now, rospy.Duration(0.1))
				(loc, rot) = self.tfList.lookupTransform('world', frame_names[node], now)
				idx = self.confignames.index(node)
				self.currconfig[idx] = Point(loc[0], loc[1])
			except Exception as e:
				#rospy.logerr(e)
				continue


		if self.waypoints:
			if self.currMove:
				#print self.currMove.point.distTo(self.currconfig[self.currMove.node_id])
				if self.currMove.point.distTo(self.currconfig[self.currMove.node_id]) < merr: #move complete
			 		if self.currMove.isAtTarget():
						self.waypoints.pop(0)
					if self.moves:
						self.setCurrMove(self.moves.pop(0))
					else:
						self.currMove = None
		self.redraw()
		self.root.after(50, self.update)
			
	#on mouse clicks, add a waypoint
	def clickCB(self, event):
		canvas = event.widget
		x = canvas.canvasx(event.x)
		y = canvas.canvasy(event.y)
		action = Waypoint.MOVE if event.num == 1 else Waypoint.VIEW
		self.planPath(Waypoint(x, y, action, Point.PX))
		self.redraw()

	#Plans the path to the next waypoint	
	def planPath(self, wp):
		self.waypoints.append(wp)
		while True:
			move = self.getNextMove(self.planconfig, wp)
			self.moves.append(move)
			move.applyTo(self.planconfig)
			#move.publish()
			if move.isAtTarget():
				break
		if not self.currMove:
			self.setCurrMove(self.moves.pop(0))

	#Returns the next major planning action
	#Rconfig: Current Configuration
	#wp: Waypoint 	
	def getNextMove(self, rconfig, wp):
		global config, ferr
		outPoint = Point() 

		#target state
		tgtAngle = rconfig[1].angleTo(wp)
		tgtDist = rconfig[1].distTo(wp) 
		if wp.action == Waypoint.VIEW: #stop short of view self.waypoints to get in ideal image range
			tgtDist -= config.view.opt
		
		#output pseudoparams
		extend = config.extend.min  #desired extension
		newtheta = tgtAngle	#init desired angle
		node_id = MajorMove.FRONT
		action = MajorMove.STEP

		#distances and angle error terms
		frontTheta = rconfig[1].angleTo(rconfig[0])
		backTheta = rconfig[2].angleTo(rconfig[1]) #math.atan2(rconfig[1].y - rconfig[2].y, rconfig[1].x - rconfig[2].x)
		frontRel = angleDiff(tgtAngle, frontTheta) #relative angle between front and target
		backRel = angleDiff(tgtAngle, backTheta)  #relative angle between back and target
		d1 = rconfig[1].distTo(rconfig[0])				 #distance between front and middle
		d2 = rconfig[1].distTo(rconfig[2]) 				 #distance between middle and rear

		if(abs(backRel) > ferr or abs(frontRel) > config.angle.max): 		#feet not pointing at the thing
			if(abs(frontRel) >= abs(backRel)): 		#rotate feet, starting with whichever is farther away
				newtheta = backTheta + math.copysign(min(abs(backRel), config.angle.max), backRel) #TODO
			else:
				newtheta = frontTheta + math.copysign(min(abs(frontRel), config.angle.max), frontRel) #TODO
				node_id = MajorMove.REAR
				extend = -config.extend.min
		else:										#move towards the thing
			if(wp.action == Waypoint.VIEW and tgtDist < config.view.max):	#view plan action
				action = MajorMove.VIEW
				extend = config.view.extend
			elif (d1 < (config.extend.max - ferr)): #extend front segment, accounting for error
				extend = config.extend.max
			elif (d2 > config.extend.min + ferr): 	#retract rear foot
				node_id = MajorMove.REAR
				extend = -config.extend.min
			else: 									#extend middle segment
				node_id = MajorMove.MIDDLE
				extend = min(tgtDist, config.extend.max - config.extend.min)

		#derive final joint coordinates
		outPoint.x = rconfig[1].x + (extend * math.cos(newtheta))
		outPoint.y = rconfig[1].y + (extend * math.sin(newtheta))
		move = MajorMove(outPoint, node_id, wp, action)
		return move

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