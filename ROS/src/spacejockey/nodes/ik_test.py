#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from collections import OrderedDict
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

#TODO: load these from URDF
joint_names = [
  'center_swivel',
  'fore_base_pitch',
  'aft_base_pitch',
  'front_pitch',
  'rear_pitch',
  'fore_extend',
  'aft_extend',
  'front_attach',
  'center_attach',
  'rear_attach']
node_names =  ['front_foot', 'center_foot', 'rear_foot']
tgt_names =  ['front_tgt', 'center_tgt', 'rear_tgt']

config = spacejockey.config("/planner")
footLocs = [(config.extend.min,0,0), (0,0,0), (-config.extend.min,0,0)]



#This is a hackackular quick and dirty implementation, needs to be fixed long term
#takes X, Y, Z tuples, returns a set of joint angles
def IK(front,center,rear):
  #initialize joint positions...
  positions = OrderedDict.fromkeys(joint_names, 0.0)
  #step one, compute detach states
  positions['front_attach'] = 0.0 if front[0] < ferr else 0.006 
  positions['center_attach'] = 0.0 if center[0] < ferr else 0.006 
  positions['rear_attach'] = 0.0 if rear[0] < ferr else 0.006 

  f_xy_dist = math.sqrt((front[0] - center[0])**2 + (front[1] - center[1])**2) - .07054 
  f_z_dist = center[2] - front[2] + .0194

  r_xy_dist = math.sqrt((center[0] - rear[0])**2 + (center[1] - rear[1])**2) - .07054
  r_z_dist = center[2] - rear[2] + .0194

  #Hacky, rotate the robot frame in the world pose
  ctheta = math.atan2(rear[0], -rear[1])
  tfCast.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(0, 0, ctheta), rospy.Time.now(), "/center_foot", "/world")

  front_theta = math.atan2(front[1] - center[1], front[0] - center[0])
  rear_theta = math.atan2(center[1] - rear[1], center[0] - rear[0])
  positions['center_swivel'] = math.atan2(math.sin(front_theta-rear_theta), math.cos(front_theta-rear_theta))
  positions['fore_extend'] = math.sqrt(f_xy_dist**2 + f_z_dist**2)
  positions['aft_extend'] = math.sqrt(r_xy_dist**2 + r_z_dist**2)

  ftheta = math.atan2(f_z_dist, f_xy_dist)
  rtheta = math.atan2(r_z_dist, r_xy_dist)

  positions['fore_base_pitch'] = -ftheta
  positions['front_pitch'] = ftheta
  positions['aft_base_pitch'] = -rtheta
  positions['rear_pitch'] = rtheta
  return positions


def makeBox():
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def make6DofMarker( name, x = 0, y = 0, z = 0):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "/world"
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

jointPub = rospy.Publisher('joint_minor', JointState)

def processFeedback(feedback):
    p = feedback.pose.position
    footLocs[tgt_names.index(feedback.marker_name)] = (p.x, p.y, p.z)
    soln = IK(footLocs[0], footLocs[1], footLocs[2])
    joint_msg = JointState()
    joint_msg.name = soln.keys()
    joint_msg.position = soln.values() 
    jointPub.publish(joint_msg)

rospy.init_node("ik_test")
tfCast = tf.TransformBroadcaster()

# create an interactive marker server on the topic namespace simple_marker
server = InteractiveMarkerServer("ik_test")
front_mrk = make6DofMarker("front_tgt", config.extend.min, 0, 0)
rear_mrk = make6DofMarker("rear_tgt", -config.extend.min, 0, 0)

server.applyChanges()
#force initial IK soln
soln = IK(*tuple(footLocs))
joint_msg = JointState()
joint_msg.name = soln.keys()
joint_msg.position = soln.values() 
jointPub.publish(joint_msg)
rospy.spin()