#!/usr/bin/env python  
import rospy
from collections import deque, namedtuple
import spacejockey
from spacejockey.kinematics import IK, joint_vel, normalize
from spacejockey.srv import *
import tf
import tf_weighted
from sensor_msgs.msg import JointState
import math
import argparse

frame_names = rospy.get_param('/planner/frame_names')
minorqueue = deque()

MinorAction = namedtuple('MinorAction', 'frame loc detach') 
ViewAction = namedtuple('ViewAction', 'range')
PauseAction = namedtuple('PauseAction', 'duration')
float_error = .0001


class MinorPlanner:
  def __init__(self, rate):
    self.Hz = rate
    self.config = spacejockey.config("/planner")
    #static view position transform
    self.view_tf_loc = (self.config.view.extend, 0.0, self.config.view.extendHeight) #TODO: maybe rename these params

    self.joint_tgt = dict()
    self.joints = dict()
    self.isPaused = False
    self.last_minor_act = None

    self.jointPub = rospy.Publisher('joint_states', JointState)    
    #rospy.Subscriber('major_actions', MajorPlanAction, self.handle_major_action)
    rospy.Subscriber('/joint_states', JointState, self.handle_joint_states)
    rospy.init_node('minor_planner')

    #set up status service
    self.status = RobotStatusResponse('idle', 0.0, 0.0)
    self.statusSrv = rospy.Service('/robot_status', RobotStatus, self.handle_status_request)

    rospy.wait_for_service('major_planner') 
    try:
      self.get_major_move = rospy.ServiceProxy('major_planner', MajorPlanner)
    except rospy.ServiceException, e:
      rospy.logerr("Service setup failed: " + str(e))
    self.last_major_id = 0

    #queue up a view movement for inital localization
    minorqueue.append(ViewAction(self.config.view.opt))
    minorqueue.append(PauseAction(rospy.Duration(3)))

    self.tfList = tf.TransformListener()
    self.tf = spacejockey.LocalTfCache(self.tfList)
    self.tfCast = tf_weighted.StaticTransformBroadcaster() #tf.TransformBroadcaster()

  def handle_status_request(self, req):
    #TODO: check time maybe?
    return self.status

  def handle_joint_states(self, msg):
    for i in range(len(msg.name)):
      self.joints[msg.name[i]] = msg.position[i]

  def is_at_joint_tgt(self):
    for j in self.joint_tgt.keys():
      if abs(self.joints[j] - self.joint_tgt[j]) > float_error:
        return False
    return True

  def eta(self):
    """estimated time until we've reached our next node"""
    times = [0.0]
    for j in self.joint_tgt.keys():
      err = abs(self.joint_tgt[j] - self.joints[j])
      times.append(err / joint_vel[j])
    return max(times) 

  def update_joints(self):
    joints = self.joints
    for j in self.joint_tgt.keys():
      err = normalize(self.joint_tgt[j] - self.joints[j]) 
      joints[j] += math.copysign(min(abs(err), (joint_vel[j] / self.Hz)), err)
    j_msg = JointState()
    j_msg.name = joints.keys()
    j_msg.position = joints.values() 
    self.jointPub.publish(j_msg)

  def unpause(self, event):
    self.isPaused = False

  def execute_minor_action(self, act):
    now = rospy.Time.now()
    if isinstance(act, PauseAction):
      self.isPaused = True
      rospy.Timer(act.duration, self.unpause, True) #oneshot timer
      if isinstance(self.last_minor_act, ViewAction):
        self.status.image_weight = 0.70 #TODO: perameterize these weights!
        self.status.locale_weight = 0.70
      return

    if isinstance(act, ViewAction):
      self.joint_tgt = IK(front = self.view_tf_loc, tgtRange = act.range)
      self.status.image_weight = 0.3
      self.status.locale_weight = 0.3
      return

    #ELSE: move action
    if act.frame == 'front_foot' and act.loc[2] > float_error:
      self.status.image_weight = 0.1
      self.status.locale_weight = 0.1
    else:
      self.status.image_weight = 0.0
      self.status.locale_weight = 0.0

    self.tf.clear()
    #pull needed values into our Transformer
    for node in frame_names.values():
      if node == act.frame:
        self.tf.saveTransform(node, 'world', act.loc, (0,0,0,1))
      else:
        self.tf.pullTransform(node, 'world') #, now) FIXME:?

    #calculate robot-frame transforms...
    (floc, foobar) = self.tf.computeTransform('front_foot', 'robot')
    (rloc, foobar) = self.tf.computeTransform('rear_foot', 'robot')
    self.joint_tgt = IK(front = floc, rear=rloc, doDetach = act.detach)

    #update the robot position...
    if act.frame == 'robot':
      t_est = now + rospy.Duration.from_sec(self.eta())
      self.tfCast.sendTransform(act.loc, (0,0,0,1), t_est, 'robot', 'world')
    self.last_minor_act = act

  def execute_move_action(self, msg):
    frame_id = frame_names[msg.node_name]
    (loc, rot) = self.tfList.lookupTransform('world', frame_id, rospy.Time(0))
    loc = list(loc)
    self.status.status_msg = 'Moving'

    #Skip detach step if node is already off the surface...
    if(loc[2] < self.config.move.detachHeight):
      loc[2] = self.config.move.detachHeight #detach
      minorqueue.append(MinorAction(frame_id, tuple(loc), True))

    loc[2] = self.config.move.clearHeight #rise to clear height
    minorqueue.append(MinorAction(frame_id, tuple(loc), False))

    resolution = 20
    dx = (1.0*msg.x-loc[0])/resolution
    dy = (1.0*msg.y-loc[1])/resolution
    for i in range(1,resolution+1):
      loc[0] = loc[0] + dx
      loc[1] = loc[1] + dy
      minorqueue.append(MinorAction(frame_id, tuple(loc), False))    

    attach_res = 10
    dz = 1.0*self.config.move.clearHeight/attach_res
    for i in range(1,attach_res+1):
      loc[2] = max(loc[2]-dz, 0.0)
      minorqueue.append(MinorAction(frame_id, tuple(loc), False)) 

  def execute_view_action(self, msg):
    #calculate range to target
    self.tf.clear()
    self.tf.pullTransform('robot', 'world')
    self.tf.saveTransform('view_tgt', 'world', (msg.x, msg.y, 0.0), (0,0,0,1))
    (vloc, foobar) = self.tf.computeTransform('view_tgt', 'robot')
    trange = math.sqrt(vloc[0]**2 + vloc[1]**2)
    self.status.status_msg = 'Viewing'

    #queue up a view movement
    minorqueue.append(ViewAction(trange))

    #queue up a pause action
    minorqueue.append(PauseAction(msg.sleep))
    return

  def loop(self):
    rate = rospy.Rate(self.Hz)
    while not rospy.is_shutdown():
      if self.isPaused:
        pass #do nothing...
      elif not self.is_at_joint_tgt():
        self.update_joints()
      elif minorqueue:
        #print('minor action')
        next = minorqueue.popleft() 
        self.execute_minor_action(next)
      else:
        move = self.get_major_move(self.last_major_id)
        self.last_major_id = move.major_id
        if move.action_type == MajorPlannerResponse.STEP:
          self.execute_move_action(move)
        elif move.action_type == MajorPlannerResponse.VIEW:
          self.execute_view_action(move)
        else: #sleep
          self.status.status_msg = 'Idle'
          minorqueue.append(PauseAction(move.sleep))
      rate.sleep()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(prog='minor_planner.py', description="this node breaks down moves into their constituent parts and executes them.")
  #parser.add_argument('-f', '--root_frame', default='world', help='the root frame_id to tie transforms to')
  parser.add_argument('-r', '--rate', type=int, default=50, help='motor control output rate (in Hz)')
  args, unknown = parser.parse_known_args()
  try:
    planner = MinorPlanner(args.rate)
    planner.loop()
  except rospy.ROSInterruptException: pass