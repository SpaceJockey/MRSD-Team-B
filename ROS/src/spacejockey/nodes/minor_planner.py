#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from collections import deque, namedtuple
import spacejockey
from spacejockey.kinematics import IK
from spacejockey.msg import MajorPlanAction
import tf
from sensor_msgs.msg import JointState
import math
import argparse



#TODO: load these from URDF
node_names =  ['front_foot', 'center_foot', 'rear_foot']
minorqueue = deque()
actionqueue = deque()

MinorAction = namedtuple('MinorAction', 'name pos detach')
PauseAction = namedtuple('PauseAction', 'name duration')
float_error = .0001

class MinorPlanner:
  def __init__(self, rate = 10):
    self.Hz = rate
    self.config = spacejockey.config("/planner")

    self.joint_tgt = dict()
    self.joints = dict()
    self.isPaused = False

    self.jointPub = rospy.Publisher('joint_states_planned', JointState)    
    rospy.Subscriber('major_actions', MajorPlanAction, self.handle_major_action)
    rospy.Subscriber('joint_states', JointState, self.handle_joint_states)
    rospy.init_node('minor_planner', anonymous=True)

    self.tf = tf.TransformListener()
    self.tfCast = tf.TransformBroadcaster()

    self.lastTf = [((self.config.extend.min,0,0), tf.transformations.quaternion_from_euler(0, 0, 0)),
                   ((0,0,0), tf.transformations.quaternion_from_euler(0, 0, 0)),
                   ((-self.config.extend.min,0,0), tf.transformations.quaternion_from_euler(0, 0, 0))]

  def handle_joint_states(self, msg):
    for i in range(len(msg.name)):
      self.joints[msg.name[i]] = msg.position[i]

  def is_at_joint_tgt(self):
    for j in self.joint_tgt.keys():
      if abs(self.joints[j] - self.joint_tgt[j]) > float_error:
        return False
    return True

  def update_joints(self):
    joints = self.joints #TODO: add joint velocities to self.joints
    j_msg = JointState()
    j_msg.name = joints.keys()
    j_msg.position = joints.values() 
    self.jointPub.publish(joint_msg)
    return True

  def unpause(self, event):
    self.isPaused = False

  def execute_minor_action(self, act):
    if act.name == 'pause':
      self.isPaused = True
      rospy.Timer(rospy.Duration(act.duration), self.unpause, True) #oneshot timer
    #TODO: pull tfs, execute joint angles
    return

  def execute_move_action(self, msg):
    node = node_names.index(msg.node_name)
    try:
      (loc, rot) = self.tf.lookupTransform('/world', msg.node_name, rospy.Time(0)) #try to get tf from world frame...
    except:
      (loc, rot) = self.lastTf[node] #fall back to internal model
    loc = list(loc)
    loc[2] = self.config.move.detachHeight #detach
    minorqueue.append(MinorAction(msg.node_name, tuple(loc), True))

    loc[2] = self.config.move.clearHeight #apex
    loc[0] = (loc[0] + msg.x) / 2.0
    loc[1] = (loc[1] + msg.y) / 2.0
    minorqueue.append(MinorAction(msg.node_name, tuple(loc), False))

    loc[2] = self.config.move.detachHeight #pre-landing
    loc[0] = msg.x
    loc[1] = msg.y
    minorqueue.append(MinorAction(msg.node_name, tuple(loc), False))

    loc[2] = 0.0 #attach
    minorqueue.append(MinorAction(msg.node_name, tuple(loc), False))
    self.lastTf[node] = ((msg.x, msg.y, 0.0), tf.transformations.quaternion_from_euler(0, 0, msg.theta))

  def execute_view_action(self, msg):
    #TODO: make this a thing!

    #queue up a pause action
    print('view action')
    minorqueue.append(PauseAction('pause', 3))
    return

  def handle_major_action(self, msg):
    actionqueue.append(msg)


  def loop(self):
    rate = rospy.Rate(self.Hz)
    while not rospy.is_shutdown():
      if self.isPaused:
        pass #do nothing...
      elif not self.is_at_joint_tgt():
        self.update_joints()
      elif minorqueue:
        print('minor action')
        next = minorqueue.popleft() 
        self.execute_minor_action(next)
      elif actionqueue:
        msg = actionqueue.popleft()
        print('major action')
        if(msg.action_type == MajorPlanAction.STEP):
          self.execute_move_action(msg)
        else:
          self.execute_view_action(msg)
      rate.sleep()

if __name__ == '__main__':
  parser = argparse.ArgumentParser(prog='minor_planner.py', description="this node breaks down moves into their constituent parts and executes them.")
  #parser.add_argument('-f', '--root_frame', default='world', help='the root frame_id to tie transforms to')
  parser.add_argument('-r', '--rate', type=int, default=10, help='motor control output rate (in Hz)')
  args, unknown = parser.parse_known_args()
  try:
    planner = MinorPlanner(args.rate)
    planner.loop()
  except rospy.ROSInterruptException: pass