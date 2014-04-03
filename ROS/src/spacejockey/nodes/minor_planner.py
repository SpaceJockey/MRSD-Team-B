#!/usr/bin/env python  
import roslib
roslib.load_manifest('spacejockey')
import rospy
from collections import OrderedDict, deque
from Queue import Queue
import spacejockey
from spacejockey.msg import MajorPlanAction, MinorPlanAction
import tf
from sensor_msgs.msg import JointState

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
movequeue = deque()
actionqueue = Queue()
#movequeue.append(IK('...'))

#rospy.loginfo(msg)

#This is a hackackular quick and dirty implementation, needs to be fixed long term
#takes X, Y, Z tuples, returns a set of joint angles
def IK(front,center,rear):
  #initialize joint positions...
  positions = OrderedDict.fromkeys(joint_names, 0.0)
  #step one, compute detach states
  positions['front_attach'] = 0.0 if front[0] < ferr else 0.006 
  positions['center_attach'] = 0.0 if center[0] < ferr else 0.006 
  positions['center_attach'] = 0.0 if rear[0] < ferr else 0.006 

  rospy.loginfo('Dumping IK: ' + str(positions))

  return positions

class MinorPlanner:
  def __init__(self):
    self.config = spacejockey.config("/planner")
    rospy.init_node('minor_planner', anonymous=True)
    rospy.Subscriber('major_actions', MajorPlanAction, self.handle_major_action)
    self.jointPub = rospy.Publisher('joint_states', JointState)
    self.tf = tf.TransformListener()
    self.tfCast = tf.TransformBroadcaster()



  def execute_move_action(self, msg):
    try:
      (ftrans, frot) = self.tf.lookupTransform('/world', '/front_foot', rospy.Time(0))
      (ctrans, crot) = self.tf.lookupTransform('/world', '/center_foot', rospy.Time(0))
      (rtrans, rrot) = self.tf.lookupTransform('/world', '/rear_foot', rospy.Time(0))

      #authoritatively repuplish nearest neighbor transform to hold robot position to world
      if(msg.node_name != 'center_foot'):
        self.tfCast.sendTransform(ctrans, crot, rospy.Time.now(), "/center_foot", "/world")
      else:
        self.tfCast.sendTransform(rtrans, rrot, rospy.Time.now(), "/rear_foot", "/world")

      node = node_names.index(msg.node_name)
      robot = [list(ftrans), list(ctrans), list(rtrans)]

      robot[node][2] = self.config.move.detachHeight #detach
      movequeue.append(IK(*tuple(robot)))

      robot[node][2] = self.config.move.clearHeight #apex
      robot[node][0] = (robot[node][0] + msg.x) / 2
      robot[node][1] = (robot[node][1] + msg.y) / 2
      movequeue.append(IK(*tuple(robot)))

      robot[node][2] = self.config.move.detachHeight #pre-landing
      robot[node][0] = msg.x
      robot[node][1] = msg.y
      movequeue.append(IK(*tuple(robot)))

      robot[node][2] = 0.0 #attach
      movequeue.append(IK(*tuple(robot)))


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      rospy.logerr('Failed to get transform: from /world to /' + msg.node_name)
      #actionqueue.appendleft(msg
    return

  def execute_view_action(self, msg):
    return

  #handle this thread safely
  def handle_major_action(self, msg):
    actionqueue.put(msg)


  def loop(self):
    r = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
      if len(movequeue) > 0:
        next = movequeue.popleft()  #next is an ordered dict...
        joint_msg = JointState()
        joint_msg.name = next.keys()
        joint_msg.position = next.values() 
        self.jointPub.publish(joint_msg)
        #TODO: delay long enough for slow joints to settle
        #TODO: publish world frame transforms...
      else:
        if not actionqueue.empty():
          msg = actionqueue.get()
          if(msg.action_type == MajorPlanAction.STEP):
            self.execute_move_action(msg)
          else:
            self.execute_view_action(msg)
      r.sleep()

if __name__ == '__main__':
    try:
        planner = MinorPlanner()
        #Wee bit hacky... may need moved somehwere...
        planner.tfCast.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "/center_foot", "/world")
        planner.loop()
    except rospy.ROSInterruptException: pass