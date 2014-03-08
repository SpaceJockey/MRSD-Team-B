#!/usr/bin/env python
from Tkinter import *
from copy import deepcopy
import math
import numpy as np
import IPython

#robot configuration constants
max_extend = 6 #linear actuator reach
min_extend = 2
max_angle = math.pi / 12.0 #maximum angle of center segment: 15 degrees

camera_foot_extend = 4 # where to put the foot when viewing
max_camera = 8 #camera view range
min_camera = 4
view_radius = 4
view_radius_square = view_radius * math.sqrt(2.0) #used for regional inspection tasks

#interpolation constants
tau = 0.0
dtau = 0.05
ferr = 0.01 #floating point error correction term

#major action number for planned moves
all_major_action = 0

#window Configs
screensize = 600, 600
screenscale = 6.0 / 100.0 #4 in = 100 px
screenorigin = 300, 300 #location of the 0 point

class Point:
	"""Point class stores and X,Y pair in inch coordinates, and provides scaling to/from the screen size, theta is optional"""
	def __init__(self, x = 0, y = 0):
		self.x = x
		self.y = y

	#x, y are screen coordinates 
	def fromScreen(self, x, y):
		global screenscale, screenorigin
		self.x = (x - screenorigin[0]) * screenscale
		self.y = (screenorigin[1] - y) * screenscale
	
	#returns outputs same values in inches relative to the world frame	
	def toScreen(self):
		global screenscale, screenorigin
		n_x = (self.x  / screenscale) + screenorigin[0]
		n_y = screenorigin[1] - (self.y / screenscale)
		return(n_x, n_y)

		#returns the pythagorean distance between two points
	def dist(self, other):
		return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

class AngledPoint(Point):
	def __init__(self, x = 0, y = 0, theta = 0):
		self.x = x
		self.y = y
		self.theta = theta
	#TODO: add angle math functions here

#robot configuration locations are represented as AngledPoints
currconfig = [AngledPoint(2.0),AngledPoint(),AngledPoint(-2.0)]
drawconfig = [AngledPoint(2.0),AngledPoint(),AngledPoint(-2.0)]
nextconfig = [AngledPoint(2.0),AngledPoint(),AngledPoint(-2.0)]

class Waypoint(Point):
    VIEW = 1
    MOVE = 2
    def __init__(self, x = 0, y = 0, action = MOVE):
		self.x = x
		self.y = y
		self.theta = 0
		self.action = action

#An ordered list of waypoint commands
waypoints = []

#returns the pythagorean distance between two points
#TODO: Deprecate this!
def pyDist(x1,y1,x2,y2):
	return math.sqrt((x1-x2)**2 + (y1-y2)**2)
	
def isAtWaypoint(rconfig, wp):
	global ferr
	if(wp.action == Waypoint.MOVE):
		return (rconfig[1][0] > wp.x - ferr and rconfig[1][0] < wp.x + ferr and rconfig[1][1] > wp.y - ferr and rconfig[1][1] < wp.y + ferr)
	else:
		return pyDist(wp.x, wp.y, rconfig[1][0], rconfig[1][1])
	
def getValidMove(rconfig, wp):
	# Rconfig: Current Configuration
	# wp: Waypoint 
	
	global max_extend, min_extend, max_angle, all_major_action, ferr
	nconfig = deepcopy(rconfig) #new configuration
	centerseg = rconfig[1];
	targetAngle = math.atan2(wp.y - centerseg[1], wp.x - centerseg[0]) 
	
	#rotate the closest way... TODO: test this
	if(centerseg[2] - targetAngle > np.pi):
		targetAngle += 2*np.pi
	elif(targetAngle - centerseg[2] > np.pi):
		targetAngle -= 2*np.pi
	
	chassisAngle = math.atan2(rconfig[0,1] - rconfig[1,1], rconfig[0,0] - rconfig[1,0]) - math.atan2(rconfig[1,1] - rconfig[2,1], rconfig[1,0] - rconfig[2,0])

	targetDist = np.sqrt( np.square(wp.x - centerseg[0]) + np.square(wp.y - centerseg[1]))
	if(targetAngle != rconfig[0,2] and chassisAngle < ferr): #front foot not pointing at the thing, so turn front foot and extend
		angle = 0
		if(rconfig[0,2] > targetAngle): #enforce angle limit
			angle =  max(targetAngle, rconfig[0,2] - max_angle)
		else:
			angle =  min(targetAngle, rconfig[0,2] + max_angle)
		nconfig[0] = [nconfig[1,0] + (min_extend * math.cos(angle)), nconfig[1,1] + (min_extend * math.sin(angle)), angle]
			
	elif(targetAngle != rconfig[2,2]): #rear foot not pointing at the thing, so turn back foot and retract
		angle = 0
		if(rconfig[0,2] > targetAngle): #enforce angle limit
			angle =  max(targetAngle, rconfig[0,2] - max_angle)
		else:
			angle =  min(targetAngle, rconfig[0,2] + max_angle)
		nconfig[2] = [nconfig[1][0] - (min_extend * math.cos(angle)), nconfig[1][1] - (min_extend * math.sin(angle)), angle]
	else:
		d1 = np.sqrt( np.square(nconfig[0,0] - nconfig[1,0]) + np.square(nconfig[0,1] - nconfig[1,1])) #distance between front and middle
		d2 = np.sqrt( np.square(nconfig[1,0] - nconfig[2,0]) + np.square(nconfig[1,1] - nconfig[2,1])) #distance between middle and rear
		if (d1 < (max_extend - ferr)): #extend front segment, accounting for error
			nconfig[0] = [nconfig[1,0] + (max_extend * math.cos(targetAngle)), nconfig[1,1] + (max_extend * math.sin(targetAngle)), targetAngle]
		elif (d2 > min_extend + ferr): #retract rear foot
			nconfig[2] = [nconfig[1][0] - (min_extend * math.cos(targetAngle)), nconfig[1][1] - (min_extend * math.sin(targetAngle)), targetAngle]
		else: #extend middle segment
			movDist = min(targetDist, max_extend - min_extend)
			nconfig[1] = [nconfig[1,0] + (movDist * math.cos(targetAngle)), nconfig[1,1] + (movDist * math.sin(targetAngle)), targetAngle]

			
	#set middle foot angle to the average of the other two
	nconfig[1,2] = (nconfig[0,2] + nconfig[2,2]) / 2.0 
	return nconfig
	
def interpolateconfigs(pconfig, nconfig, val):
	return pconfig * (1.0 - val) + nconfig * val

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
			cTh = 4 * math.cos(p.theta)
			sTh = 4 * math.sin(p.theta)
			self.canvas.create_line(x - cTh, y + sTh, x + cTh, y - sTh, fill=color, tags=tag)
			
	def update(self):
		global waypoints, currconfig, nextconfig, drawconfig, tau, dtau
		#self.canvas.delete(ALL) # remove all items
		if(len(waypoints) == 0):
			self.status.set("IDLE")
		else: #Update robot motion 
			self.status.set("MOVING")
			if isAtWaypoint(currconfig, waypoints[0]):
				waypoints.pop(0)
				tau = 0.0
				self.root.after(20, self.update)
				return
			elif (tau >= 1.0): #get next move
				currconfig = nextconfig
				nextconfig = getValidMove(currconfig, waypoints[0])
				drawconfig = currconfig
				tau = 0.0
			else:
				tau += dtau
				drawconfig = interpolateconfigs(currconfig, nextconfig, tau)
			

		#redraw waypoints
		self.canvas.delete("waypoint") # redraw the config every cycle
		for w in waypoints:
			color = "green" if w.action == Waypoint.MOVE else "red"
			self.drawPoint(w,color,"waypoint")				
			
		#draw Robot config
		self.canvas.delete("foot") # redraw the config every cycle
		for f in drawconfig:
			self.drawPoint(f,"blue","foot")

		#reschedule task
		self.root.after(15, self.update)
	
	#on mouse clicks, add a waypoint
	def clickCB(self, event):
		global waypoints
		canvas = event.widget
		x = canvas.canvasx(event.x)
		y = canvas.canvasy(event.y)
		x,y = screenToIn(x,y)
		action = Waypoint.MOVE if event.num == 1 else Waypoint.VIEW
		waypoints.append(Waypoint(x,y,action))

#set up window
root = Tk()   
root.wm_title("Planner Prototype")
root.config(background="#000000")
app = App(root)

#open a terminal to mess around with stuff
#IPython.embed()

#start the GUI
root.mainloop()
