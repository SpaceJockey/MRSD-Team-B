#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
import spacejockey
import tf
from sensor_msgs.msg import JointState
import math
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *

import IPython


#floating point error compensation
ferr = 0.0001

joints = spacejockey.urdf.joint_map

#unroll joint limits from the URDF
joint_upper = dict()
joint_lower = dict()
safe_limits = rospy.get_param('/use_smallest_joint_limits')
#TODO: unroll these using lambda defs...
for j in joints.keys():
  try:
    if(safe_limits and (joints[j].type == "revolute" or joints[j].type == "prismatic")):
      try:
        joint_upper[j] = joints[j].safety_controller.soft_upper_limit
        joint_lower[j] = joints[j].safety_controller.soft_lower_limit
      except AttributeError:
        joint_upper[j] = joints[j].limit.upper
        joint_lower[j] = joints[j].limit.lower
    else:
      joint_upper[j] = joints[j].limit.upper
      joint_lower[j] = joints[j].limit.lower
  except AttributeError:
    continue

node_names =  ['front_foot', 'center_foot', 'rear_foot']
tgt_names =  ['front_tgt', 'center_tgt', 'rear_tgt']

config = spacejockey.config("/planner")
footLocs = [(config.extend.min,0,0), (0,0,0), (-config.extend.min,0,0)]

def clip_limits(positions):
  for j in positions.keys():
    try:
      if positions[j] > joint_upper[j]:
        positions[j] = joint_upper[j]
      if positions[j] < joint_lower[j]:
        positions[j] = joint_lower[j]
    except KeyError:
      continue
  return positions

#This is a hackackular quick and dirty implementation, needs to be fixed long term
#takes X, Y, Z tuples (in the local robot frame), returns a partial set of joint angles
def IK(front = None, rear = None, doDetach = False):
  #initialize joint positions...
  positions = dict() #.fromkeys(joints.keys(), 0.0)
  #tfCast.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/center_foot", "/robot")

  positions['center_attach'] = 0.0 #default off...
  if(front):
    f_xy_dist = math.sqrt(front[0]**2 + front[1]**2) - .07054 #TODO:, parameterize joint offsets from URDF...
    f_z_dist = front[2] + .0194
    positions['fore_extend'] = math.sqrt(f_xy_dist**2 + f_z_dist**2)
    ftheta = math.atan2(f_z_dist, f_xy_dist)
    positions['fore_base_pitch'] = -ftheta
    positions['front_pitch'] = ftheta
    front_theta = math.atan2(front[1], front[0])
    positions['center_swivel'] = front_theta
    if doDetach:
      if front[2] < 0.0: #detach the center foot
        positions['center_attach'] = 1.0 
      if front[2] > ferr: #twist the front foot to detach
        positions['front_pitch'] += 0.09

  if(rear):
    r_xy_dist = math.sqrt(rear[0]**2 + rear[1]**2) - .07054
    r_z_dist = rear[2] + .0194
    positions['aft_extend'] = math.sqrt(r_xy_dist**2 + r_z_dist**2)
    rtheta = math.atan2(r_z_dist, r_xy_dist)
    positions['aft_base_pitch'] = -rtheta
    positions['rear_pitch'] = rtheta
    rear_theta = math.atan2(-rear[1], -rear[0]) #TODO: verify this...
    positions['center_swivel'] = rear_theta
    if doDetach:
      if rear[2] < 0.0: #detach the center foot
        positions['center_attach'] = 1.0 
      if rear[2] > ferr: #twist the rear foot to detach
        positions['rear_pitch'] += 0.09

  #TODO: this may not be right!
  if(front and rear):
    #Hacky, rotate the robot frame in the world pose
    ctheta = math.atan2(rear[0], -rear[1])
    #tfCast.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0, 0, ctheta), rospy.Time.now(), "/center_foot", "/robot")
    positions['center_swivel'] = math.atan2(math.sin(front_theta-rear_theta), math.cos(front_theta-rear_theta))

  return clip_limits(positions)

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

def makeXYRMarker( name, x = 0, y = 0, z = 0, parent = "/world"):
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
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = "move_y"
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  control.orientation_mode = InteractiveMarkerControl.FIXED
  int_marker.controls.append(control)

  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = "rotate_z"
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  control.orientation_mode = InteractiveMarkerControl.FIXED
  int_marker.controls.append(control)

  server.insert(int_marker, processFeedback)
  return int_marker

jointPub = rospy.Publisher('joint_minor', JointState)

def poseToTf(pose):
  return ((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

def processFeedback(feedback):
    #update marker location
    markers[feedback.marker_name].pose = feedback.pose;
    #republish all marker poses, then get the transforms between them...
    for m in markers.itervalues():
      (loc, rot) = poseToTf(m.pose)
      #print(rot) #TODO: debug this...
      tfCast.sendTransform(loc, rot, rospy.Time.now(), m.name, m.header.frame_id)

    try:
      tfList.waitForTransform(feedback.marker_name, "/robot",  rospy.Time(0), rospy.Duration(.1));
      (loc, rot) = tfList.lookupTransform(feedback.marker_name, "/robot",  rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      print(e)
      return
    soln = {
      'front_tgt' : IK(front = loc, doDetach = True),
      'rear_tgt' : IK(rear = loc, doDetach = True),
      'robot' : {}
    }[feedback.marker_name]

    joint_msg = JointState()
    joint_msg.name = soln.keys()
    joint_msg.position = soln.values() 
    jointPub.publish(joint_msg)

rospy.init_node("ik_test")

# IPython.embed()
tfCast = tf.TransformBroadcaster()
tfList = tf.TransformListener()
# create an interactive marker server on the topic namespace simple_marker
server = InteractiveMarkerServer("ik_test")
markers = {
  'robot'     : makeXYRMarker("robot", 0, 0, 0), 
  'front_tgt' : make6DofMarker("front_tgt", config.extend.min, 0, 0),
  'rear_tgt'  : make6DofMarker("rear_tgt", -config.extend.min, 0, 0)
}

server.applyChanges()
#force initial IK soln
soln = IK(*tuple(footLocs))
joint_msg = JointState()
joint_msg.name = soln.keys()
joint_msg.position = soln.values() 
jointPub.publish(joint_msg)
rospy.spin()