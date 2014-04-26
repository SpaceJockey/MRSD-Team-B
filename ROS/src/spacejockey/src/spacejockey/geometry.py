import math
from . import msg, config

window = config("/gui")

class Point:
	"""Point class stores and X,Y pair in meters, and provides scaling to/from the screen size, theta is optional"""
	PX = 1
	M = 2
	def __init__(self, x = 0, y = 0, units = M):
		if (units == Point.PX):
			self.x = (x - window.origin.x) * window.scale
			self.y = (window.origin.y - y) * window.scale
		else:
			self.x = x
			self.y = y

	#returns output values in screen coordinates
	def toScreen(self):
		n_x = (self.x  / window.scale) + window.origin.x
		n_y = window.origin.y - (self.y / window.scale)
		return (int(n_x), int(n_y))

	#returns the pythagorean distance between two points
	def distTo(self, other):
		return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

	#returns the global bearing of the other point
	def angleTo(self, other):
		return math.atan2(other.y - self.y, other.x - self.x) 

#returns the difference between two angles
def angleDiff(x, y): 
	return math.atan2(math.sin(x-y), math.cos(x-y))

class Waypoint(Point, msg.Waypoint):
	def __init__(self, x = 0, y = 0, action = msg.Waypoint.MOVE, units = Point.M):
		msg.Waypoint.__init__(self, 0, 0, action) #set up action type
		Point.__init__(self, x, y, units) #use unit casting to get things in meters...