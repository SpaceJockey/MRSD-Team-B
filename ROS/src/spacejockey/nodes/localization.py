#!/usr/bin/env python
import roslib
import rospy
import spacejockey
import tf
import tf_weighted
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from ar_track_alvar.msg import AlvarMarkers
import argparse

if __name__ == '__main__':
	rospy.init_node('localization')
	sCast = tf_weighted.StaticTransformBroadcaster()
	wCast = tf_weighted.WeightedTransformBroadcaster()
	cache = spacejockey.LocalTfCache()

	#initialize robot pose and static marker locations in the world...
	try:
		staticTfs = rospy.get_param('/static_tfs')
		now = rospy.Time.now()
		sTFs = {}
		for t in staticTfs.iteritems():
			sTF = TransformStamped()   
			sTF.header.frame_id = 'world'
			sTF.header.stamp = now
			sTF.child_frame_id = t[0]
			sTF.transform.translation = Vector3(*tuple(t[1][0]))
			sTF.transform.rotation = Quaternion(*tuple(t[1][1]))
			sTFs[t[0]] = sTF
		sCast.sendTransforms(sTFs.values())
		del sTFs['robot']
	except:
		pass
	rospy.spin()

"""
	def handle_markers(msg):
		now = msg.header.stamp
		weight = 0.5 #TODO: pull this from robot movement state publisher!
		cache.clear()
		cache.pullTransform('robot', 'camera', now)
		#load in static marker poses
		for t in sTFs.values():
			t.header.stamp = cache.tfTime
			cache.setTransform(t)

		tfs = []
		for marker in msg.markers:
			sid = 'marker_' + str(marker.id)
			pos = marker.pose.pose.position
			rot = marker.pose.pose.orientation
			cache.pullTransform('ar_' + sid, 'camera')
			#cache.saveTransform('camera', sid,  (pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w))
			#sCast.sendTransform((pos.x, pos.y, pos.z), (rot.x, rot.y, rot.z, rot.w), rospy.Time.now(), 'test_marker', 'camera') #sid)
			loc, rot = cache.computeTransform('robot', 'ar_' + sid) #world')
			sCast.sendTransform(loc, rot, now, 'robot', 'world')

	rospy.Subscriber('/ar_pose_marker', AlvarMarkers, handle_markers)
	rospy.spin()
	"""