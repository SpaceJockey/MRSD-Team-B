#!/usr/bin/env python
import roslib
import rospy
import spacejockey
import tf
import tf_weighted
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
import argparse

if __name__ == '__main__':
	rospy.init_node('localization')
	sCast = tf_weighted.StaticTransformBroadcaster()
	wCast = tf_weighted.WeightedTransformBroadcaster()

	#initialize robot pose and static marker locations in the world...
	try:
		staticTfs = rospy.get_param('/static_tfs')
		now = rospy.Time.now()
		sTFs = []
		for t in staticTfs.iteritems():
			sTF = TransformStamped()   
			sTF.header.frame_id = 'world'
			sTF.header.stamp = now
			sTF.child_frame_id = t[0]
			sTF.transform.translation = Vector3(*tuple(t[1][0]))
			sTF.transform.rotation = Quaternion(*tuple(t[1][1]))
			sTFs.append(sTF)
		sCast.sendTransforms(sTFs)
	except KeyError:
		pass
	rospy.spin()