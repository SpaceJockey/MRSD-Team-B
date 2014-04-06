#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from spacejockey.msg import MajorPlanAction, MinorPlanAction

import tf


tfCast = tf.TransformBroadcaster()

def handle_minor_action(msg):
  tfCast.sendTransform(msg.tf.transform.translation,
    msg.tf.transform.rotation,
    rospy.Time.now(),
    "/plan/" + str(msg.major_id) + "/" + str(msg.minor_id) + "/" + msg.tf.child_frame_id,
    "world")

def handle_major_action(msg):
  tfCast.sendTransform((msg.x, msg.y, 0),
    tf.transformations.quaternion_from_euler(0, 0, msg.theta),
    rospy.Time.now(),
    "/plan/" + str(msg.major_id) + "/" + msg.node_name,
    "world")

if __name__ == '__main__':
  rospy.init_node('planner_tf_broadcaster')
  rospy.Subscriber('major_actions',
    MajorPlanAction,
    handle_major_action)
  rospy.Subscriber('minor_actions',
    MinorPlanAction,
    handle_minor_action)
  rospy.spin()