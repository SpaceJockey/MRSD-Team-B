#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from collections import deque, namedtuple
import spacejockey
from spacejockey.kinematics import IK, joint_vel
from spacejockey.msg import MajorPlanAction
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, TransformStamped
import math
import argparse



#TODO: load these from URDF
frame_names=  rospy.get_param('/joints/frame_names')
minorqueue = deque()
actionqueue = deque()

MinorAction = namedtuple('MinorAction', 'name loc detach') #todo: add view angles
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

    self.tf = tf.Transformer(True,  cache_time = rospy.Duration(1)) #local transformer for maths...
    self.tfList = tf.TransformListener()
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

  def eta(self):
    """estimated time untile we've reached our next node"""
    times = [0.0]
    for j in self.joint_tgt.keys():
      err = self.joint_tgt[j] - self.joints[j]
      times.append(joint_vel[j] * err)
    return max(times) 

  def update_joints(self):
    joints = self.joints #TODO: add joint velocities to self.joints
    for j in self.joint_tgt.keys():
      err = self.joint_tgt[j] - self.joints[j] 
      joints[j] += math.copysign(min(abs(err), (joint_vel[j] / self.Hz)), err)
    j_msg = JointState()
    j_msg.name = joints.keys()
    j_msg.position = joints.values() 
    self.jointPub.publish(j_msg)
    return True

  def unpause(self, event):
    self.isPaused = False


  def saveTransform(self, child_id, parent_id, loc, rot, time = rospy.Time(0)):
    """save a tf to our private Transformer"""
    t = TransformStamped()
    t.header.frame_id = parent_id
    t.header.stamp = time
    t.child_frame_id = child_id
    t.transform.translation.x = loc[0]
    t.transform.translation.y = loc[1]
    t.transform.translation.z = loc[2]
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]
    self.tf.setTransform(t)

  def pullTransform(self, child_id, parent_id, time = rospy.Time(0)):
    """pull a tf from the world to our private Transformer"""
    (loc, rot) = self.tfList.lookupTransform(parent_id, child_id, time)
    self.saveTransform(child_id, parent_id, loc, rot)
    return (loc, rot)

  def computeTransform(self, child_id, parent_id, time = rospy.Time(0)):
    """query our private Transformer"""
    return self.tf.lookupTransform(parent_id, child_id, time)

  def execute_minor_action(self, act):
    if act.name == 'pause':
      self.isPaused = True
      rospy.Timer(rospy.Duration(act.duration), self.unpause, True) #oneshot timer
      return
    self.tf.clear()

    #pull needed values into our Transformer
    for node in frame_names.keys():
      if node == act.name:
        self.saveTransform(frame_names[node], 'world', act.loc, (0,0,0,1))
      else:
        self.pullTransform(frame_names[node], 'world')

    #calculate robot-frame transforms...
    (floc, foobar) = self.computeTransform('front_foot', 'robot')
    (rloc, foobar) = self.computeTransform('rear_foot', 'robot')
    self.joint_tgt = IK(front = floc, rear=rloc, doDetach = act.detach)

    #update the robot position...
    if act.name == 'center_foot':
      t_est = rospy.Duration.from_sec(self.eta())
      self.tfCast.sendTransform(act.loc, (0,0,0,1), rospy.Time.now() + t_est, 'static/robot', 'world')

  def execute_move_action(self, msg):
    (loc, rot) = self.tfList.lookupTransform('/world', msg.node_name, rospy.Time(0)) #try to get tf from world frame...
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