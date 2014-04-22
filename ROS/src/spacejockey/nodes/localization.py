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
	#initialize robot pose and marker locations in the world...
	caster = tf_weighted.WeightedTransformBroadcaster(True)
	try:
		staticTfs = rospy.get_param('/static_tfs')
		now = rospy.Time.now()
		wTFs = []
		for t in staticTfs.iteritems():
			wTF = TransformStamped()      #TeeHee!
			wTF.header.frame_id = 'world'
			wTF.header.stamp = now
			wTF.child_frame_id = t[0]
			wTF.transform.translation = Vector3(*tuple(t[1][0]))
			wTF.transform.rotation = Quaternion(*tuple(t[1][1]))
			wTFs.append(wTF)
		caster.sendTransforms(wTFs, 1.0)
	except KeyError:
		pass
	rospy.spin()