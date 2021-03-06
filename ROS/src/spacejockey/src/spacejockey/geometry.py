import math

class Point(object):
	"""Point class stores and X,Y pair in meters"""
	def __init__(self, x = 0, y = 0):
			self.x = x
			self.y = y

	#returns the pythagorean distance between two points
	def distTo(self, other):
		return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)

	#returns the global bearing of the other point
	def angleTo(self, other):
		return math.atan2(other.y - self.y, other.x - self.x) 

	def __repr__(self):
		return str((self.x, self.y))

#returns the difference between two angles
def angleDiff(x, y): 
	return normalizeAngle(x-y) #math.atan2(math.sin(x-y), math.cos(x-y))


def normalizeAngle(angle):
  """normalize an angle to the range -pi - pi"""
  if angle > math.pi:
    angle -= 2 * math.pi
  elif angle < -math.pi:
    angle += 2 * math.pi
  return angle