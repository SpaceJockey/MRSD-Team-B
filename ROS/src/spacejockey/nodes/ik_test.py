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
def IK(center, front = None, rear = None, doDetach = False):
  #initialize joint positions...
  positions = dict() #.fromkeys(joints.keys(), 0.0)
  tfCast.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/center_foot", "/robot")

  if(front):
    f_xy_dist = math.sqrt((front[0] - center[0])**2 + (front[1] - center[1])**2) - .07054 
    f_z_dist = center[2] - front[2] + .0194
    positions['fore_extend'] = math.sqrt(f_xy_dist**2 + f_z_dist**2)
    ftheta = math.atan2(f_z_dist, f_xy_dist)
    positions['fore_base_pitch'] = -ftheta
    positions['front_pitch'] = ftheta
    front_theta = math.atan2(front[1] - center[1], front[0] - center[0])
    positions['center_swivel'] = front_theta
    positions['center_attach'] = 0.0 if front[2] < 0.0 else 0.006 
    if (doDetach and front[2] > ferr): #twist the front foot off!
      positions['front_pitch'] += 0.09

  if(rear):
    r_xy_dist = math.sqrt((center[0] - rear[0])**2 + (center[1] - rear[1])**2) - .07054
    r_z_dist = center[2] - rear[2] + .0194
    positions['aft_extend'] = math.sqrt(r_xy_dist**2 + r_z_dist**2)
    rtheta = math.atan2(r_z_dist, r_xy_dist)
    positions['aft_base_pitch'] = -rtheta
    positions['rear_pitch'] = rtheta
    rear_theta = math.atan2(center[1] - rear[1], center[0] - rear[0])
    positions['center_swivel'] = rear_theta
    positions['center_attach'] = 0.0 if rear[2] < 0.0 else 0.006
    if (doDetach and rear[2] > ferr): #twist the front foot off!
      positions['rear_pitch'] += 0.09

  #TODO: this may not be right!
  if(front and rear):
    #Hacky, rotate the robot frame in the world pose
    ctheta = math.atan2(rear[0], -rear[1])
    tfCast.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0, 0, ctheta), rospy.Time.now(), "/center_foot", "/robot")
    positions['center_swivel'] = math.atan2(math.sin(front_theta-rear_theta), math.cos(front_theta-rear_theta))

  return clip_limits(positions)

def make6DofMarker( name, x = 0, y = 0, z = 0, parent = "/world"):
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = parent
  int_marker.scale = 0.1

  int_marker.name = name
  int_marker.description = name

  int_marker.pose.position.x = x
  int_marker.pose.position.y = y
  int_marker.pose.position.z = z

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
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = parent
  int_marker.scale = 0.1

  int_marker.name = name
  int_marker.description = name

  int_marker.pose.position.x = x
  int_marker.pose.position.y = y
  int_marker.pose.position.z = z

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
    #tfCast.sendTransform(ctrans, crot, rospy.Time.now(), "/center_foot", "/world")
    (loc, rot) = poseToTf(feedback.pose)
    tfCast.sendTransform(loc, rot, rospy.Time.now(), feedback.marker_name, feedback.header.frame_id)
    if feedback.marker_name == 'robot':
      return

    #tfList.lookupTransform("/base_link", "/camera_depth_optical_frame",ros::Time::now(), transform);
    footLocs[tgt_names.index(feedback.marker_name)] = loc
    soln = IK(footLocs[1], footLocs[0], footLocs[2])
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
robot_mrk = makeXYRMarker("robot", 0, 0, 0)
tfCast.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), '/robot', '/world') #init robot frame
front_mrk = make6DofMarker("front_tgt", config.extend.min, 0, 0) #, "/robot")
rear_mrk = make6DofMarker("rear_tgt", -config.extend.min, 0, 0) #, "/robot")

server.applyChanges()
#force initial IK soln
soln = IK(*tuple(footLocs))
joint_msg = JointState()
joint_msg.name = soln.keys()
joint_msg.position = soln.values() 
jointPub.publish(joint_msg)
rospy.spin()