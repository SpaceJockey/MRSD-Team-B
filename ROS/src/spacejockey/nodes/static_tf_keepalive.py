#!/usr/bin/env python
import roslib
roslib.load_manifest('spacejockey')
import rospy
import spacejockey
import tf
import argparse

#this node is a workaround for tf's poor support for static transforms
#TODO: figure out how to publish these to the underlying tf2 framework...
if __name__ == '__main__':
	parser = argparse.ArgumentParser(prog='static_tf_keepalive.py', description="this node is a workaround for tf's poor support for static transforms")
	parser.add_argument('-f', '--root_frame', default='world', help='the root frame_id to tie transforms to')
	parser.add_argument('-r', '--rate', type=int, default=100, help='publishing rate to rebroadcast stuff at.')
	args, unknown = parser.parse_known_args()

	rospy.init_node('static_tf_keepalive')
	listener = tf.TransformListener()
	caster = tf.TransformBroadcaster()
	staticTfs = rospy.get_param('/static_tfs')

	rate = rospy.Rate(args.rate)
	while not rospy.is_shutdown():
		#listen for new static transforms
		for frame in listener.getFrameStrings():
			if not frame.startswith('static/'):
				continue
			try:
				t = listener.lookupTransform(args.root_frame, frame, rospy.Time(0))
				staticTfs[frame[7:]] = t
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
				rospy.logerr(e)
				continue

		#republish all known static tfs...
		
		for t in staticTfs.iteritems():
			caster.sendTransform(t[1][0], t[1][1], rospy.Time.now(), t[0], args.root_frame)
			continue

		rate.sleep()
