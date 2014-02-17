#!/usr/bin/python
from Tkinter import *
from copy import deepcopy
import math
import numpy as np
import IPython

#robot configuration constants
max_extend = 6
min_extend = 2

#maximum angle of center segment
max_angle = math.pi / 12.0 #15 degrees


#foot locations are represented as (X,Y, theta) tuples theta is relative to the x axis
currconfig = np.array([[2.0,0,0],[0.0,0,0],[-2.0,0,0]])
nextconfig = np.array([[2.0,0,0],[0.0,0,0],[-2.0,0,0]])
drawconfig = np.array([[2.0,0,0],[0.0,0,0],[-2.0,0,0]])
tau = 0.0
dtau = 0.05
ferr = 0.01 #floating point error correction term

#Waypoint locations for the center foot in X,Y tuples (don't care about theta)
waypoints = []

#window Configs
screensize = 600, 600
screenscale = 6.0 / 100.0 #4 in = 100 px
screenorigin = 300, 300 #location of the 0 point

#x, y are screen coordinates outputs same values in inches relative to the world frame	
def screenToIn(x, y):
	global screenscale, screenorigin
	n_x = (x - screenorigin[0]) * screenscale
	n_y = (screenorigin[1] - y) * screenscale
	return(n_x, n_y)
	
def inToScreen(x, y):
	global screenscale, screenorigin
	n_x = (x  / screenscale) + screenorigin[0]
	n_y = screenorigin[1] - (y / screenscale)
	return(n_x, n_y)
	
def isAtWaypoint(rconfig, wp):
	global ferr
	return (rconfig[1][0] > wp[0] - ferr and rconfig[1][0] < wp[0] + ferr and rconfig[1][1] > wp[1] - ferr and rconfig[1][1] < wp[1] + ferr)
	
def getValidMove(rconfig, wp):
	global max_extend, min_extend, max_angle, ferr
	nconfig = deepcopy(rconfig) #new configuration
	centerseg = rconfig[1];
	targetAngle = math.atan2(wp[1] - centerseg[1], wp[0] - centerseg[0]) 
	
	#rotate the closest way... ToDo: test this
	if(centerseg[2] -  targetAngle > np.pi):
		targetAngle += 2*np.pi
	elif(targetAngle - centerseg[2] > np.pi):
		targetAngle -= 2*np.pi
	
	chassisAngle = math.atan2(rconfig[0,1] - rconfig[1,1], rconfig[0,0] - rconfig[1,0]) - math.atan2(rconfig[1,1] - rconfig[2,1], rconfig[1,0] - rconfig[2,0])
	#print(targetAngle, chassisAngle)
	targetDist = np.sqrt( np.square(wp[0] - centerseg[0]) + np.square(wp[1] - centerseg[1]))

	if(targetAngle != rconfig[0,2] and chassisAngle < ferr): #front foot not pointing at the thing, so turn front foot and extend
		angle = 0
		if(rconfig[0,2] > targetAngle): #enforce angle limit
			angle =  max(targetAngle, rconfig[0,2] - max_angle)
		else:
			angle =  min(targetAngle, rconfig[0,2] + max_angle)
		nconfig[0] = [nconfig[1,0] + (max_extend * math.cos(angle)), nconfig[1,1] + (max_extend * math.sin(angle)), angle]
			
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
			#print("extend front")
		elif (d2 > min_extend + ferr): #retract rear foot
			nconfig[2] = [nconfig[1][0] - (min_extend * math.cos(targetAngle)), nconfig[1][1] - (min_extend * math.sin(targetAngle)), targetAngle]
			#print("retract rear")
		else: #extend middle segment
			movDist = min(targetDist, max_extend - min_extend)
			nconfig[1] = [nconfig[1,0] + (movDist * math.cos(targetAngle)), nconfig[1,1] + (movDist * math.sin(targetAngle)), targetAngle]
			#print("extend middle")

			
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
		
		self.status = StringVar()
		self.status.set("IDLE")
		self.statusbox = Label(frame, textvariable = self.status, relief=SUNKEN, width = 20);
		self.statusbox.pack(side=RIGHT)
		
		#update GUI every 20 ms
		self.root.after(20, self.update)
		
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
			dims = inToScreen(*w)
			self.canvas.create_oval(dims[0]-2,dims[1]-2,dims[0]+2,dims[1]+2, outline="green", tags="waypoint")				
			
		#draw Robot config
		self.canvas.delete("foot") # redraw the config every cycle
		for f in drawconfig:
			dims = inToScreen(f[0], f[1])
			self.canvas.create_oval(dims[0]-4, dims[1]-4, dims[0]+4, dims[1]+4, outline="blue", tags="foot")
			self.canvas.create_line(dims[0] - (6 * math.cos(f[2])), dims[1] + (6 * math.sin(f[2])), dims[0] + (6 * math.cos(f[2])), dims[1] - (6 * math.sin(f[2])), fill="blue", tags="foot")
		
		#reschedule task
		self.root.after(15, self.update)
	
	#on mouse clicks, add a waypoint
	def clickCB(self, event):
		global waypoints
		canvas = event.widget
		x = canvas.canvasx(event.x)
		y = canvas.canvasy(event.y)
		waypoints.append(screenToIn(x,y))
		self.canvas.create_oval(x-2,y-2,x+2,y+2, outline="green", tags="waypoint") #draw the waypoint

#set up window
root = Tk()   
root.wm_title("Planner Prototype")
root.config(background="#000000")
app = App(root)

#open a terminal to mess around with stuff
#IPython.embed()

#start the GUI
root.mainloop()
