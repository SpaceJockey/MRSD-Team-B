#!/usr/bin/env python
from Tkinter import *
from copy import deepcopy
import math
import IPython

#ROS Imports
import roslib
#roslib.load_manifest('spacejockey')
import rospy
import sys
from spacejockey.msg import PlannerAction

#get configuration constants from the param server
config = rospy.get_param("/planner")

#interpolation constants
tau = 0.0
dtau = 0.05
ferr = 0.0001 #floating point error correction term
draw_view = False

#major action number for planned moves
major_action_id = 0

#window Configs
screensize = 600, 600
screenscale = 6.0 / 100.0 #4 in = 100 px
screenorigin = 300, 300 #location of the 0 point

class Point:
	"""Point class stores and X,Y pair in inch coordinates, and provides scaling to/from the screen size, theta is optional"""
	PX = 1
	IN = 2
	def __init__(self, x = 0, y = 0, units = IN):
		global screenscale, screenorigin
		if (units == Point.PX):
			self.x = (x - screenorigin[0]) * screenscale
			self.y = (screenorigin[1] - y) * screenscale
		else:
			self.x = x
			self.y = y

	#returns output values in screen coordinates
	def toScreen(self):
		global screenscale, screenorigin
		n_x = (self.x  / screenscale) + screenorigin[0]
		n_y = screenorigin[1] - (self.y / screenscale)
		return(n_x, n_y)

	#returns the pythagorean distance between two points
	def distTo(self, other):
		return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

	#returns the global bearing of the other point
	def angleTo(self, other):
		return math.atan2(other.y - self.y, other.x - self.x) 

class AngledPoint(Point):
	def __init__(self, x = 0, y = 0, theta = 0, units = Point.IN):
		Point.__init__(self, x, y, units)
		self.theta = theta

	def __repr__(self):
		return '[' + "{:.3f}".format(self.x) + ', ' + "{:.3f}".format(self.y) + ', ' + "{:.4f}".format(self.theta) + ']'


#robot configuration locations are represented as tuples of AngledPoints representing each foot.
#(front, middle, rear)
currconfig = (AngledPoint(2.0),AngledPoint(),AngledPoint(-2.0))
drawconfig = (AngledPoint(2.0),AngledPoint(),AngledPoint(-2.0))
nextconfig = (AngledPoint(2.0),AngledPoint(),AngledPoint(-2.0))

class Waypoint(Point):
    VIEW = 1
    MOVE = 2

    def __init__(self, x = 0, y = 0, action = MOVE, units = Point.IN):
		Point.__init__(self, x,y,units)
		self.action = action

#This is the FIFO queue of waypoints
#implemented with a list
waypoints = []

#returns the pythagorean distance between two points
#TODO: Deprecate this!
def pyDist(x1,y1,x2,y2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)
	
def isAtWaypoint(rconfig, wp):
	global ferr, config
	if(wp.action == Waypoint.MOVE):
		return rconfig[1].distTo(wp) < ferr
	else:
		return config['min_camera'] > rconfig[1].distTo(wp) < config['max_camera']
	
#Returns the next major planning action
#Rconfig: Current Configuration
#wp: Waypoint 	
def getValidMove(rconfig, wp):
	global config, major_action_id, ferr, draw_view
	nconfig = deepcopy(rconfig) #copy new configuration from old
	tgtAngle = rconfig[1].angleTo(wp)
	tgtDist = rconfig[1].distTo(wp) if wp.action == Waypoint.MOVE else rconfig[1].distTo(wp) - config['opt_camera']
	chasAngle = rconfig[0].theta - rconfig[2].theta
	action = "STEP"
	draw_view = False

	if(tgtAngle != rconfig[0].theta): #front foot not pointing at the thing, so turn front foot
		angle = tgtAngle
		#TODO: enforce angle limits
		#if(rconfig[0,2] > tgtAngle): #enforce angle limit
		#	angle =  max(tgtAngle, rconfig[0,2] - config['max_angle'])
		#else:
		#	angle =  min(tgtAngle, rconfig[0,2] + config['max_angle'])
		nconfig[0].x = rconfig[1].x + (config['min_extend'] * math.cos(angle))
		nconfig[0].y = rconfig[1].y + (config['min_extend'] * math.sin(angle))
		nconfig[0].theta = angle
			
	elif(tgtAngle != rconfig[2].theta): #rear foot not pointing at the thing, so turn back foot and retract
		angle = tgtAngle
		#TODO: enforce angle limits
		#if(rconfig[0,2] > tgtAngle): #enforce angle limit
		#	angle =  max(tgtAngle, rconfig[0,2] - config['max_angle'])
		#else:
		#	angle =  min(tgtAngle, rconfig[0,2] + config['max_angle'])
		nconfig[2].x = rconfig[1].x + (config['min_extend'] * math.cos(angle))
		nconfig[2].y = rconfig[1].y + (config['min_extend'] * math.sin(angle))
		nconfig[2].theta = angle
	else:
		d1 = nconfig[1].distTo(nconfig[0]) #distance between front and middle
		d2 = nconfig[1].distTo(nconfig[2]) #distance between middle and rear
		if(wp.action == Waypoint.VIEW and tgtDist < config['max_camera']):	#view plan action
			nconfig[0].x = nconfig[1].x + (config['camera_extend'] * math.cos(tgtAngle))
			nconfig[0].y = nconfig[1].y + (config['camera_extend'] * math.sin(tgtAngle))
			draw_view = True
			action = "VIEW"
		elif (d1 < (config['max_extend'] - ferr)): #extend front segment, accounting for error
			nconfig[0].x = nconfig[1].x + (config['max_extend'] * math.cos(tgtAngle))
			nconfig[0].y = nconfig[1].y + (config['max_extend'] * math.sin(tgtAngle))
		elif (d2 > config['min_extend'] + ferr): #retract rear foot
			nconfig[2].x = nconfig[1].x - (config['min_extend'] * math.cos(tgtAngle))
			nconfig[2].y = nconfig[1].y - (config['min_extend'] * math.sin(tgtAngle))
		else: #extend middle segment
			movDist = min(tgtDist, config['max_extend'] - config['min_extend'])
			nconfig[1].x = nconfig[1].x + (movDist * math.cos(tgtAngle))
			nconfig[1].y = nconfig[1].y + (movDist * math.sin(tgtAngle))
	#set middle foot angle to the average of the other two
	nconfig[1].theta = (nconfig[0].theta + nconfig[2].theta) / 2.0 

	print("Major action #" + `major_action_id` + ": " + action + " " + `nconfig`) #TODO: output this to a ROS topic
	major_action_id = major_action_id + 1
	return nconfig

#main application class
class App:
	
	def __init__(self, master):
		global screensize, waypoints
		self.root = master
		frame = Frame(master)
		frame.pack()
		
		self.prevX = 0
		self.prevY = 0

		self.canvas = Canvas(frame, bg="black", height = screensize[1], width = screensize[0], confine = "true", cursor = "cross")
		self.canvas.pack(side=TOP)
		self.canvas.bind("<Button-1>", self.clickCB) 
		self.canvas.bind("<Button-3>", self.clickCB) 
		
		self.status = StringVar()
		self.status.set("IDLE")
		self.statusbox = Label(frame, textvariable = self.status, relief=SUNKEN, width = 20);
		self.statusbox.pack(side=RIGHT)
		
		#update GUI every 20 ms
		self.root.after(20, self.update)

	def drawPoint(self, p, color = "blue", tag = ""):
		x,y = p.toScreen()
		self.canvas.create_oval(x-3,y-3,x+3,y+3, outline=color, tags=tag)
		if(isinstance(p, AngledPoint)):
			cTh = 6 * math.cos(p.theta)
			sTh = 6 * math.sin(p.theta)
			self.canvas.create_line(x - cTh, y + sTh, x + cTh, y - sTh, fill=color, tags=tag)
			
	def update(self):
		global waypoints, currconfig, nextconfig, drawconfig, tau, dtau, draw_view

		#reschedule task
		self.root.after(15, self.update)
		
		#update robot movement plan
		if(len(waypoints) == 0):
			self.status.set("Idle")
		else:
			self.status.set("Moving")
			if isAtWaypoint(currconfig, waypoints[0]):
				waypoints.pop(0)
			elif (tau < (1.0 - ferr)):
				tau += dtau
				#drawconfig = interpolateconfigs(currconfig, nextconfig, tau) TODO: fix this
			else:
				currconfig = nextconfig
				nextconfig = getValidMove(currconfig, waypoints[0])
				drawconfig = currconfig
				tau = 0.0
				
		#redraw everything
		self.canvas.delete(ALL) 

		#redraw waypoints
		for w in waypoints:
			color = "green" if w.action == Waypoint.MOVE else "red"
			self.drawPoint(w,color,"waypoint")				
			
		#draw Robot config
		for f in drawconfig:
			self.drawPoint(f,"blue","foot")

		#TODO: add camera view drawing and pause code
		#if (draw_view):
		#	print("viewing...")
			
	#on mouse clicks, add a waypoint
	def clickCB(self, event):
		global waypoints
		canvas = event.widget
		x = canvas.canvasx(event.x)
		y = canvas.canvasy(event.y)
		action = Waypoint.MOVE if event.num == 1 else Waypoint.VIEW
		waypoints.append(Waypoint(x, y, action, Point.PX))

#set up window
root = Tk()   
root.wm_title("Planner Prototype")
root.config(background="black")
app = App(root)

#open a terminal to mess around with stuff
#IPython.embed()

#start the GUI
root.mainloop()
