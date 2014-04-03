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
import math

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
  positions['rear_attach'] = 0.0 if rear[0] < ferr else 0.006 

  f_xy_dist = math.sqrt((front[0] - center[0])**2 + (front[1] - center[1])**2) - .07054 
  f_z_dist = center[2] - front[2] - .0194

  r_xy_dist = math.sqrt((center[0] - rear[0])**2 + (center[1] - rear[1])**2) - .07054
  r_z_dist = center[2] - rear[2] - .0194

  front_theta = math.atan2(front[1] - center[1], front[0] - center[0])
  rear_theta = math.atan2(center[1] - rear[1], center[0] - rear[0])
  positions['center_swivel'] = math.atan2(math.sin(front_theta-rear_theta), math.cos(front_theta-rear_theta))
  positions['fore_extend'] = math.sqrt(f_xy_dist**2 + f_z_dist**2)
  positions['aft_extend'] = math.sqrt(r_xy_dist**2 + r_z_dist**2)

  ftheta = math.atan2(f_z_dist, f_xy_dist)
  rtheta = math.atan2(r_z_dist, r_xy_dist)

  positions['fore_base_pitch'] = ftheta
  positions['front_pitch'] = -ftheta
  positions['aft_base_pitch'] = rtheta
  positions['rear_pitch'] = -rtheta
  return positions

class MinorPlanner:
  def __init__(self):
    self.config = spacejockey.config("/planner")
    self.jointPub = rospy.Publisher('joint_minor', JointState)
    rospy.Subscriber('major_actions', MajorPlanAction, self.handle_major_action)
    rospy.init_node('minor_planner', anonymous=True)
    self.tf = tf.TransformListener()
    self.tfCast = tf.TransformBroadcaster()

    self.lastTf = [((self.config.extend.min,0,0), tf.transformations.quaternion_from_euler(0, 0, -1.57)),
                   ((0,0,0), tf.transformations.quaternion_from_euler(0, 0, -1.57)),
                   ((-self.config.extend.min,0,0), tf.transformations.quaternion_from_euler(0, 0, -1.57))]






  def execute_move_action(self, msg):
    try:
      (ftrans, frot) = self.lastTf[0] #self.tf.lookupTransform('/world', '/front_foot', rospy.Time(0))
      (ctrans, crot) = self.lastTf[1] #self.tf.lookupTransform('/world', '/center_foot', rospy.Time(0))
      (rtrans, rrot) = self.lastTf[2] #self.tf.lookupTransform('/world', '/rear_foot', rospy.Time(0))

      #authoritatively repuplish nearest neighbor transform to hold robot position to world
      #TODO: fix this...
      #if(msg.node_name != 'center_foot'):
      self.tfCast.sendTransform(ctrans, crot, rospy.Time.now(), "/center_foot", "/world")
      #else:
      #  self.tfCast.sendTransform(rtrans, rrot, rospy.Time.now(), "/rear_foot", "/world")

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
      self.lastTf[node_names.index(msg.node_name)] = ((msg.x, msg.y, 0.0), tf.transformations.quaternion_from_euler(0, 0, msg.theta))


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
      rospy.sleep(1)

if __name__ == '__main__':
    try:
        planner = MinorPlanner()
        #Wee bit hacky... may need moved somehwere...
        #planner.tfCast.sendTransform((0,0,0), (0,0,0,1), rospy.Time.now(), "/center_foot", "/world")
        planner.loop()
    except rospy.ROSInterruptException: pass