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
import numpy as np

frame_names = rospy.get_param('/planner/frame_names')
minorqueue = deque()

MinorAction = namedtuple('MinorAction', 'frame loc detach') 
ViewAction = namedtuple('ViewAction', 'loc range')
PauseAction = namedtuple('PauseAction', 'duration')
float_error = .0001


class MinorPlanner:
  def __init__(self, rate):
    self.Hz = rate
    self.config = spacejockey.config("/planner")

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
    minorqueue.append(ViewAction((self.config.view.extend, 0.0, self.config.view.extendHeight), self.config.view.opt))
    minorqueue.append(PauseAction(rospy.Duration(3)))

    self.tfList = tf.TransformListener()
    self.tf = spacejockey.LocalTfCache(self.tfList)

    #hacktastic!
    self.tfList.waitForTransform('front_foot', 'world', rospy.Time(0), rospy.Duration(1))
    self.tf.pullTransform('front_foot', 'world')
    self.tf.pullTransform('rear_foot', 'world')
    self.tf.pullTransform('robot', 'world')

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
    prev_act = self.last_minor_act
    self.last_minor_act = act
    
    if isinstance(act, PauseAction):
      self.isPaused = True
      rospy.Timer(act.duration, self.unpause, True) #oneshot timer
      if isinstance(prev_act, ViewAction):
        self.status.image_weight = 0.70 #TODO: perameterize these weights!
        self.status.locale_weight = 0.70

      return

    if isinstance(act, ViewAction):
      rloc, foobar = self.tf.computeTransform('rear_foot', 'robot')
      self.joint_tgt = IK(act.loc, rloc, tgtRange = act.range)
      self.status.image_weight = 0.2
      self.status.locale_weight = 0.3
      return

    #ELSE: move action
    if act.frame == 'front_foot' and act.loc[2] > float_error:
      self.status.image_weight = 0.0
      self.status.locale_weight = 0.1
    else:
      self.status.image_weight = 0.0
      self.status.locale_weight = 0.0

    #pull needed value into our Transformer
    for node in frame_names.values():
      if node == act.frame:
        self.tf.saveTransform(node, 'world', act.loc, (0,0,0,1))

    #calculate robot-frame transforms...
    (floc, foobar) = self.tf.computeTransform('front_foot', 'robot')
    (rloc, foobar) = self.tf.computeTransform('rear_foot', 'robot')

    self.joint_tgt = IK(floc, rloc, doDetach = act.detach)

    if act.frame == 'robot': #update the robot position...
      self.tfCast.sendTransform(act.loc, (0,0,0,1), now, 'robot', 'world')

  #TODO: make this support splines??
  def interpolate_move(self, frame_id, start, end, steps, doDetach):
    locs = np.array([start, end])
    steps = np.linspace(0.0, 1.0, steps)
    for i in steps:
      loc = np.average(locs, axis=0, weights=[1.0-i, i])
      minorqueue.append(MinorAction(frame_id, tuple(loc), doDetach))
    return end


  def execute_move_action(self, msg):
    frame_id = frame_names[msg.node_name]
    (loc, rot) = self.tfList.lookupTransform('world', frame_id, rospy.Time(0))
    loc = list(loc)
    self.status.status_msg = 'Moving'

    #Skip detach step if node is already off the surface...
    if(loc[2] < self.config.move.detachHeight):
      loc = self.interpolate_move(frame_id, loc, [loc[0], loc[1], self.config.move.clearHeight], 5, True)

    loc = self.interpolate_move(frame_id, loc, [msg.x, msg.y, self.config.move.clearHeight], 10, False)
    loc = self.interpolate_move(frame_id, loc, [msg.x, msg.y, 0.0], 7, False)


  def execute_view_action(self, msg):
    #detach
    loc, rot = self.tfList.lookupTransform('world', 'front_foot', rospy.Time(0))
    if(loc[2] < self.config.move.detachHeight):
      loc = self.interpolate_move('front_foot', loc, [loc[0], loc[1], self.config.move.detachHeight], 3, True)

    #calculate range to target
    loc, rot = self.tf.computeTransform('robot', 'world')
    trange = math.sqrt((loc[0] - msg.x)**2 + (loc[1] - msg.y)**2)

    theta = math.atan2(msg.y - loc[1], msg.x - loc[0])

    #static view position transform
    extend = self.config.view.extend
    v_loc = (loc[0] + math.cos(theta)*extend, loc[0] + math.sin(theta)*extend, self.config.view.extendHeight)
    self.status.status_msg = 'Viewing'

    #queue up a view movement
    minorqueue.append(ViewAction(v_loc, trange))

    #queue up a pause action
    minorqueue.append(PauseAction(msg.sleep))

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
    rospy.loginfo('Minor Planner Online')
    planner.loop()
  except rospy.ROSInterruptException: pass