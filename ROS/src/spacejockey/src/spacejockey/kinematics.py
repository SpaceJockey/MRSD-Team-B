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

def clip_limits(joint_pos):
  for j in joint_pos.keys():
    try:
      if joint_pos[j] > joint_upper[j]:
        joint_pos[j] = joint_upper[j]
      if joint_pos[j] < joint_lower[j]:
        joint_pos[j] = joint_lower[j]
    except KeyError:
      continue
  return joint_pos



#TODO: parameterize link offsets from URDF!
def IK(front, rear = None, doDetach = False, checkLimits = True, tgtRange = None):
  """Takes X, Y, Z tuples (in the local robot frame), and returns a partial set of joint angles"""

  #pull in critical dimensions from urdf:
  try:
    urdf_center_z = urdf.joint_map['robot_rot'].origin.position[2]
    urdf_front_z = urdf.joint_map['front_attach'].origin.position[2]
    urdf_front_x = urdf.joint_map['front_attach'].origin.position[0]
    urdf_fore_base_x = urdf.joint_map['fore_base_pitch'].origin.position[0]
    urdf_cam_y = urdf.joint_map['camera_fixed'].origin.position[1]
    #urdf_cam_theta = urdf.joint_map['camera_fixed'].origin.rotation[1] - (math.pi / 2.0)
    urdf_cam_theta = math.radians(20) #FIXME!
  except KeyError, e:
    raise Exception('Malformed URDF joint configuration: ' + str(e))

  #calculate offsets
  z_offset = urdf_center_z + urdf_front_z
  xy_offset = urdf_front_x + urdf_fore_base_x

  #init output dictionary
  joint_pos = dict()
  joint_pos['center_attach'] = 0.0
  joint_pos['center_swivel'] = 0.0

  #throw errors
  assert not (tgtRange and doDetach), "Detachment and range arguments must not be used at the same time"

  f_xy_dist = math.sqrt(front[0]**2 + front[1]**2) - xy_offset
  f_z_dist = front[2] - z_offset
  joint_pos['fore_extend'] = math.sqrt(f_xy_dist**2 + f_z_dist**2)
  ptheta = math.atan2(f_z_dist, f_xy_dist)
  joint_pos['fore_base_pitch'] = -ptheta
  joint_pos['front_pitch'] = ptheta
  
  ftheta = math.atan2(front[1], front[0])
  
  if doDetach:
    if front[2] < -detach_height + float_err: #detach the center foot
      joint_pos['center_attach'] = 0.1 
    if front[2] >= detach_height - float_err: #twist the front foot to detach
      joint_pos['front_pitch'] += 0.1

  if(rear):
    r_xy_dist = math.sqrt(rear[0]**2 + rear[1]**2) - xy_offset
    r_z_dist = rear[2] - z_offset
    joint_pos['aft_extend'] = math.sqrt(r_xy_dist**2 + r_z_dist**2)
    ptheta = math.atan2(r_z_dist, r_xy_dist)
    joint_pos['aft_base_pitch'] = ptheta
    joint_pos['rear_pitch'] = -ptheta

    rtheta = math.atan2(-rear[1], -rear[0])
    joint_pos['robot_rot'] = rtheta
    joint_pos['center_swivel'] = ftheta - rtheta

    if doDetach:
      if rear[2] < -detach_height + float_err: #detach the center foot
        joint_pos['center_attach'] = 0.1 
      if rear[2] >= detach_height - float_err: #twist the rear foot to detach
        joint_pos['rear_pitch'] += 0.1

  #point the camera at the target
  if tgtRange:
    tgtTheta = math.atan2(f_z_dist, tgtRange - f_xy_dist) - urdf_cam_theta
    joint_pos['front_pitch'] += tgtTheta
    oTheta = math.atan(urdf_cam_y/tgtRange)
    joint_pos['center_swivel'] -= oTheta

  #clip to limits and return
  if(checkLimits):
  	return clip_limits(joint_pos)
  else:
  	return joint_pos