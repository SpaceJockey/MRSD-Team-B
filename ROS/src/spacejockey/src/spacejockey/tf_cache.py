import rospy
import tf
from geometry_msgs.msg import TransformStamped

class LocalTfCache(tf.Transformer):
	def __init__(self, listener = None):
		tf.Transformer.__init__(self, True,  cache_time = rospy.Duration(1)) #local transformer for maths...
		self.Listener = listener or tf.TransformListener()
		self.tfTime = rospy.Time(0) #use the same timestamp for all internal tf calcs

	def saveTransform(self, child_id, parent_id, loc, rot):
		"""save a tf to our private Transformer"""
		t = TransformStamped()
		t.header.frame_id = parent_id
		t.header.stamp = self.tfTime
		t.child_frame_id = child_id
		t.transform.translation.x = loc[0]
		t.transform.translation.y = loc[1]
		t.transform.translation.z = loc[2]
		t.transform.rotation.x = rot[0]
		t.transform.rotation.y = rot[1]
		t.transform.rotation.z = rot[2]
		t.transform.rotation.w = rot[3]
		self.setTransform(t)

	def pullTransform(self, child_id, parent_id, time = rospy.Time(0)):
		"""pull a tf from the world to our private Transformer"""
		self.Listener.waitForTransform(parent_id, child_id, time, rospy.Duration(0.1))
		(loc, rot) = self.Listener.lookupTransform(parent_id, child_id, time)
		self.saveTransform(child_id, parent_id, loc, rot)
		return (loc, rot)

	def computeTransform(self, child_id, parent_id):
		"""query our private Transformer"""
		return self.lookupTransform(parent_id, child_id, self.tfTime)