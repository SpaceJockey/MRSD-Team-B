#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from spacejockey.msg import MajorPlanAction, MinorPlanAction

import tf

def execute_move_action(msg):
  return

def execute_view_action(msg):
  return

def handle_major_action(msg):
  tfCast.sendTransform(msg.tf.transform.translation,
    msg.tf.transform.rotation,
    rospy.Time.now(),
    "/plan/" + str(msg.major_id) + "/" + str(msg.minor_id) + "/" + msg.tf.child_frame_id,
    "world")


if __name__ == '__main__':
  rospy.init_node('minor_planner')
  rospy.Subscriber('major_actions',
    MajorPlanAction,
    handle_major_action)
  rospy.spin()