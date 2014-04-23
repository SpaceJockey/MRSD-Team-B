#!/usr/bin/env python  
import roslib
#roslib.load_manifest('spacejockey')
import rospy
from collections import deque, namedtuple
import spacejockey
from spacejockey.kinematics import IK, joint_vel, normalize
from spacejockey.srv import *
import tf
import tf_weighted
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform, TransformStamped
import math
import argparse

#TODO: load these from URDF
frame_names = rospy.get_param('/planner/frame_names')
minorqueue = deque()
actionqueue = deque()

MinorAction = namedtuple('MinorAction', 'name loc detach') 
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

    rospy.init_node('minor_planner')
    self.jointPub = rospy.Publisher('joint_states', JointState)   
    rospy.Subscriber('/joint_states', JointState, self.handle_joint_states)

    rospy.wait_for_service('major_planner') 
    try:
      self.get_major_move = rospy.ServiceProxy('major_planner', MajorPlanner)
    except rospy.ServiceException, e:
      rospy.logerr("Service setup failed: " + str(e))
    self.last_major_id = 0

    self.tf = tf.Transformer(True,  cache_time = rospy.Duration(1)) #local transformer for maths...
    self.tfTime = rospy.Time(0) #use the same timestamp for all internal tf calcs
    self.tfList = tf.TransformListener()
    #self.tfCast = tf.TransformBroadcaster()
    self.tfCast = tf_weighted.WeightedTransformBroadcaster()
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
    """estimated time until we've reached our next node"""
    times = [0.0]
    for j in self.joint_tgt.keys():
      err = abs(self.joint_tgt[j] - self.joints[j])
      times.append(err/joint_vel[j])
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

  def saveTransform(self, child_id, parent_id, loc, rot):
    """save a tf to our private Transformer"""
    t = TransformStamped()
    t.header.frame_id = parent_id
    t.header.stamp = self.tfTime
    t.child_frame_id = child_id
    t.transform.translation.x = loc[0]
    t.transform.translation.y = loc[1]
    t.transform.translation.z = loc[2]
    t.transform.rotation.x = rot[0]
    t.transform.rotation.y = rot[1]
    t.transform.rotation.z = rot[2]
    t.transform.rotation.w = rot[3]
    self.tf.setTransform(t)

  def pullTransform(self, child_id, parent_id):
    """pull a tf from the world to our private Transformer"""
    (loc, rot) = self.tfList.lookupTransform(parent_id, child_id, rospy.Time(0))
    self.saveTransform(child_id, parent_id, loc, rot)
    return (loc, rot)

  def computeTransform(self, child_id, parent_id):
    """query our private Transformer"""
    return self.tf.lookupTransform(parent_id, child_id, self.tfTime)

  def execute_minor_action(self, act):
    now = rospy.Time.now()
    if isinstance(act, PauseAction):
      self.isPaused = True
      rospy.Timer(rospy.Duration(act.duration), self.unpause, True)
      return

    if isinstance(act, ViewAction):
      self.joint_tgt = IK(front = self.view_tf_loc, tgtRange = act.range)
      return

    #move action...
    self.tf.clear()

    #choose a bound joint
    #bframe = ''
    #bloc = None
    #brot = None

    #pull needed values into our Transformer
    for frame in frame_names.values():
      if frame == act.name:
        self.saveTransform(frame, 'world', act.loc, (0,0,0,1))
      else:
        bframe = frame #use this as the fixed frame
        bloc, brot = self.pullTransform(frame, 'world')

    #TODO: Not sure why this completely breaks things... maybe not worth looking into it...
    #self.tfCast.sendTransform((0, 0, 0), (0,0,0,1), now, act.name, 'world', 0.0)  #unbind mobile tf
    #self.tfCast.sendTransform(bloc, brot, now, bframe, 'world', 1.0)                 #bind fixed tf

    #calculate robot-frame transforms...
    (floc, foobar) = self.computeTransform('front_foot', 'robot')
    (rloc, foobar) = self.computeTransform('rear_foot', 'robot')
    self.joint_tgt = IK(front = floc, rear=rloc, doDetach = act.detach)

    #update the robot position...
    if act.name == 'robot':
      t_est = now + rospy.Duration.from_sec(self.eta())
      self.tfCast.sendTransform(act.loc, (0,0,0,1), t_est, 'robot', 'world', 1.0)

  def execute_move_action(self, msg):
    frame_id = frame_names[msg.node_name] #TODO: recast minor actions to use frame names...
    (loc, rot) = self.tfList.lookupTransform('world', frame_id, rospy.Time(0))
    loc = list(loc)

    #Skip detach step if node is already off the surface...
    if(loc[2] < self.config.move.detachHeight):
      loc[2] = self.config.move.detachHeight #detach
      minorqueue.append(MinorAction(msg.node_name, tuple(loc), True))

    loc[2] = self.config.move.clearHeight #rise to clear height
    minorqueue.append(MinorAction(msg.node_name, tuple(loc), False))

    resolution = 20
    for i in range(1,resolution+1):
      factor = 1.0*i/resolution
      loc[0] = (1-factor)*loc[0] + factor*msg.x
      loc[1] = (1-factor)*loc[1] + factor*msg.y
      minorqueue.append(MinorAction(msg.node_name, tuple(loc), False))    

    attach_res = 10
    for i in range(1,attach_res+1):
      factor = 1.0-(1.0*i/attach_res)
      loc[2] = factor*self.config.move.clearHeight
      minorqueue.append(MinorAction(msg.node_name, tuple(loc), False)) 

  def execute_view_action(self, msg):
    #calculate range to target
    self.tf.clear()
    self.pullTransform('robot', 'world')
    self.saveTransform('view_tgt', 'world', (msg.x, msg.y, 0.0), (0,0,0,1))
    (vloc, foobar) = self.computeTransform('view_tgt', 'robot')
    trange = math.sqrt(vloc[0]**2 + vloc[1]**2)

    #queue up a view movement
    minorqueue.append(ViewAction(trange))

    #queue up a pause action
    minorqueue.append(PauseAction(3)) #TODO: perameterize pause duration...
    return

  def handle_major_action(self, msg):
    actionqueue.append(msg)


  def loop(self):
    rate = rospy.Rate(self.Hz)
    i = 0
    while not rospy.is_shutdown():
      i += 1
      if self.isPaused:
        #print "\tis paused"
        #TODO: add weight outputs...
        pass #do nothing...
      elif not self.is_at_joint_tgt():
        #print "\tupdate joints"
        self.update_joints()
      elif minorqueue:
        next = minorqueue.popleft()
        #print "minor: " + str(len(minorqueue)) 
        self.execute_minor_action(next)
        #print "done"
      else:
        move = self.get_major_move(self.last_major_id)
        self.last_major_id = move.major_id
        if move.action_type == MajorPlannerResponse.STEP:
          self.execute_move_action(move)
        elif move.action_type == MajorPlannerResponse.VIEW:
          self.execute_view_action(move)
        else: #sleep
          minorqueue.append(PauseAction(1))
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
