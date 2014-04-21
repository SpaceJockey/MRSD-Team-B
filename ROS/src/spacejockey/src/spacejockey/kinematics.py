import roslib
#roslib.load_manifest('spacejockey')
import rospy
import math
from . import urdf

#floating point error compensation
float_err = 0.0001

#unroll joint limits from the URDF
joint_upper = dict()
joint_lower = dict()
joint_vel = dict()
free_joints = []

safe_limits = rospy.get_param('/use_smallest_joint_limits')
detach_height = rospy.get_param('/planner/move/detachHeight')
#TODO: unroll these using lambda defs...
for j in urdf.joint_map.keys():
  try:
    joint_vel[j] = urdf.joint_map[j].limit.velocity
    j_type = urdf.joint_map[j].type
    if not (j_type == "prismatic" or j_type == "revolute"): 
      continue

    free_joints.append(j)
    if safe_limits:  
      try:
        joint_upper[j] = urdf.joint_map[j].safety_controller.soft_upper_limit
        joint_lower[j] = urdf.joint_map[j].safety_controller.soft_lower_limit
      except AttributeError:
        joint_upper[j] = urdf.joint_map[j].limit.upper
        joint_lower[j] = urdf.joint_map[j].limit.lower
    else:
      joint_upper[j] = urdf.joint_map[j].limit.upper
      joint_lower[j] = urdf.joint_map[j].limit.lower
  except AttributeError:
    continue

def normalize(angle):
  """normalize an angle to the range -pi - pi"""
  if angle > math.pi:
    angle -= math.pi
  elif angle < -math.pi:
    angle += math.pi
  return angle

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

#TODO: parameterize link offsets from URDF!
def IK(front = None, rear = None, doDetach = False, checkLimits = True, tgtRange = None):
  """Takes X, Y, Z tuples (in the local robot frame), and returns a partial set of joint angles"""
  positions = dict()
  positions['center_attach'] = 0.0 #default off...
  if(front):
    f_xy_dist = math.sqrt(front[0]**2 + front[1]**2) - .07054
    f_z_dist = front[2] - .0194
    positions['fore_extend'] = math.sqrt(f_xy_dist**2 + f_z_dist**2)
    ptheta = math.atan2(f_z_dist, f_xy_dist)
    positions['fore_base_pitch'] = -ptheta
    positions['front_pitch'] = ptheta
    
    ftheta = math.atan2(front[1], front[0])
    #positions['robot_rot'] = 0.0
    positions['center_swivel'] = ftheta
    
    if doDetach:
      if front[2] < -detach_height + float_err: #detach the center foot
        positions['center_attach'] = 0.1 
      if front[2] >= detach_height - float_err: #twist the front foot to detach
        positions['front_pitch'] -= 0.1
    elif tgtRange: #FIXME: implement viewing angle for view moves
      tgtTheta = math.atan2(f_z_dist, tgtRange - f_xy_dist) - math.radians(20) #TODO: pull camera angle from URDF
      positions['front_pitch'] += tgtTheta

  if(rear):
    r_xy_dist = math.sqrt(rear[0]**2 + rear[1]**2) - .07054
    r_z_dist = rear[2] - .0194
    positions['aft_extend'] = math.sqrt(r_xy_dist**2 + r_z_dist**2)
    ptheta = math.atan2(r_z_dist, r_xy_dist)
    positions['aft_base_pitch'] = ptheta
    positions['rear_pitch'] = -ptheta

    rtheta = math.atan2(-rear[1], -rear[0])
    positions['robot_rot'] = rtheta
    positions['center_swivel'] = -rtheta

    if doDetach:
      if rear[2] < -detach_height + float_err: #detach the center foot
        positions['center_attach'] = 0.1 
      if rear[2] >= detach_height - float_err: #twist the rear foot to detach
        positions['rear_pitch'] += 0.1

  #TODO: what happens to the center joint if both front and back are set?
  if front and rear:
    positions['center_swivel'] = ftheta - rtheta

  if(checkLimits):
  	return clip_limits(positions)
  else:
  	return positions