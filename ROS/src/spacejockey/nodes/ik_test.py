#!/usr/bin/env python  
import roslib
#roslib.load_manifest('spacejockey')
import rospy
import spacejockey
from spacejockey.kinematics import IK
import tf
from sensor_msgs.msg import JointState
import math
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

config = spacejockey.config("/planner")

def makeMarker(name, x, y, z, parent):
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = parent
  int_marker.scale = 0.1

  int_marker.name = name
  int_marker.description = name

  int_marker.pose.position.x = x
  int_marker.pose.position.y = y
  int_marker.pose.position.z = z

  int_marker.pose.orientation.x = 0.0
  int_marker.pose.orientation.y = 0.0
  int_marker.pose.orientation.z = 0.0
  int_marker.pose.orientation.w = 1.0
  return int_marker


def make6DofMarker( name, x = 0, y = 0, z = 0, parent = "/world"):
  int_marker = makeMarker(name, x, y, z, parent)
  
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = "move_x"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.orientation_mode = InteractiveMarkerControl.FIXED
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = "move_z"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.orientation_mode = InteractiveMarkerControl.FIXED
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = "move_y"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.orientation_mode = InteractiveMarkerControl.FIXED
  int_marker.controls.append(control)

  server.insert(int_marker, processFeedback)
  return int_marker

jointPub = rospy.Publisher('planner/joint_states', JointState)

def poseToTf(pose):
  return ((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

def processFeedback(feedback):
    #update marker location
    (loc, rot) = poseToTf(feedback.pose)
    tfCast.sendTransform(loc, rot, rospy.Time(0), '/static/' + feedback.marker_name, feedback.header.frame_id)

    try:
      (floc, frot) = tfList.lookupTransform('robot', 'front_tgt', rospy.Time(0))
      (rloc, rrot) = tfList.lookupTransform('robot', 'rear_tgt', rospy.Time(0))
      #tfCast.sendTransform(floc, frot, rospy.Time.now(), '/brain', '/robot') #hollaback
      (cloc, crot) = tfList.lookupTransform('/world', '/robot', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      print(e)
      return
    if feedback.marker_name == 'view':
      tgt = math.sqrt((loc[0] - cloc[0])**2 + (loc[1] - cloc[1])**2)
      soln = IK(rear = rloc, front = floc, tgtRange = tgt)
    else:
      soln = IK(rear = rloc, front = floc, doDetach = True)
    publishJoints(soln)
    #TODO: republish the robot frame rotated...



def publishJoints(ik_soln):
    joint_msg = JointState()
    joint_msg.name = ik_soln.keys()
    joint_msg.position = ik_soln.values() 
    jointPub.publish(joint_msg)

rospy.init_node("ik_test")

# IPython.embed()
tfCast = tf.TransformBroadcaster()
tfList = tf.TransformListener()
# create an interactive marker server on the topic namespace simple_marker
server = InteractiveMarkerServer("ik_test")
markers = {
  'robot': make6DofMarker("robot", 0, 0, 0), 
  'front_tgt' : make6DofMarker("front_tgt", config.extend.min, 0, 0),
  'rear_tgt'  : make6DofMarker("rear_tgt", -config.extend.min, 0, 0),
  'view'  : make6DofMarker("view", config.view.opt, 0, 0)
}
server.applyChanges()

#publish initial marker tfs
#for m in markers.iteritems():
#  (loc, rot) = poseToTf(m[1].pose)
#  tfCast.sendTransform(loc, rot, rospy.Time(0.1), '/static/' + m[0], m[1].header.frame_id)
#rospy.sleep(.1)

rospy.spin()