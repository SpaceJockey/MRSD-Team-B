#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from spacejockey.msg import PlannerAction

import tf


tfCast = tf.TransformBroadcaster()

def handle_planner_action(msg):
  tfCast.sendTransform((msg.x, msg.y, 0),
    tf.transformations.quaternion_from_euler(0, 0, msg.theta),
    rospy.Time.now(),
    "/major_plan/" + msg.node_name,
    "world")

if __name__ == '__main__':
  rospy.init_node('planner_tf_broadcaster')
  rospy.Subscriber('major_actions',
    PlannerAction,
    handle_planner_action)
  rospy.spin()