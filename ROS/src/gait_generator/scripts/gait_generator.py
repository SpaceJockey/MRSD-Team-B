#!/usr/bin/env python
import rospy, time, roslib, os, rospkg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from rqt_sj.msg import GaitComm
from math import pi, radians, degrees, cos, sin, tan, sqrt


CENTER_SWIVEL = 0
CENTER_PITCHF = 1
CENTER_PITCHR = 2
FRONT_PITCH = 3
REAR_PITCH = 4
CENTER_PRISMF = 5
CENTER_PRISMR = 6

def get_prism_ang(dist, clear):
  prism = sqrt(dist*dist + clear*clear)
  ang = tan(clear/dist)
  return prism, ang

class GaitGenerator():
    prism_len = 0.15
    clearance = 0.01
    lift_res = 20
    xtend_res = 40
    max_xtend = 0.10
    turn_res = 25
    
    x_dist = sqrt((max_xtend + prism_len)*(max_xtend + prism_len) - clearance*clearance)
    
    dsb_joints = [0.0,]*12
    joint_msg = JointState()
    joint_msg.name = ['center_swivel','fore_base_pitch','aft_base_pitch','front_pitch','rear_pitch','fore_extend','aft_extend']
    joint_msg.position = dsb_joints[0:7]
    joint_msg.velocity=[0.0,]*7

    schedPub = rospy.Publisher('sched_joints',JointState)
    jointPub = rospy.Publisher('joint_states',JointState)
    
    def __init__(self):
      self.listener()
      print("Gait Generator Online.")
      rospy.spin()
      
    def command_receive(self,msg):
      print(msg.data)
      data = msg.data.split()
      if data[0] == "clearance":
        self.clearance = float(data[1])
        self.x_dist = sqrt((self.max_xtend + self.prism_len)*(self.max_xtend + self.prism_len) - self.clearance*self.clearance)
      elif data[0] == "liftres":
        self.lift_res = int(data[1])
      elif data[0] == "extendres":
        self.xtend_res = int(data[1])
      elif data[0] == "maxextend":
        self.max_xtend = float(data[1])
        self.x_dist = sqrt((self.max_xtend + self.prism_len)*(self.max_xtend + self.prism_len) - self.clearance*self.clearance)
      elif data[0] == "turnres":
        self.turn_res = int(data[1])
      else:
        if data[1] == 'front' or data[1] == 'rear':
          if data[1] == 'front':
            center_pitch = CENTER_PITCHF
            end_pitch = FRONT_PITCH
            prism = CENTER_PRISMF
          elif data[1] == 'rear':
            center_pitch = CENTER_PITCHR
            end_pitch = REAR_PITCH
            prism = CENTER_PRISMR    
                      
          if data[0] == 'lift' or data[0] == 'place':
            lift = data[0] == 'lift'
            extended = data[2] == 'extended'
            self.execute_lift_place(center_pitch,end_pitch,prism, extended, lift)
          elif data[0] == 'extend' or data[0] == 'retract':
            extend = data[0] == 'extend'
            self.execute_extend_retract(center_pitch,end_pitch,prism, extend)
          elif data[0] == 'turn' or data[0] == 'straighten':
            turn = data[0] == 'turn'
            self.execute_turn_straighten(radians(float(data[2])), turn)
            
        elif data[1] == 'middle':
          if data[0] == 'lift' or data[0] == 'place':
            self.execute_middle_lift_place(data[2], data[0]=='lift')  
          elif data[0] == 'extend':
            self.execute_middle_extend_retract(data[2])      


    
    def listener(self):
      rospy.init_node('gait_generator', anonymous=True)
      rospy.Subscriber("gait_commands", String, self.command_receive)

    def updateJointState(self):
        time.sleep(0.01)
        self.joint_msg.header.stamp = rospy.Time.now()
        self.joint_msg.position = self.dsb_joints[0:7]
        self.schedPub.publish(self.joint_msg)

    # Executes a lift or place operation
    # center_pitch: the slider widget for the appropriate center segment pitch joint
    # end_pitch: '' end segment pitch joint
    # center_prism: '' prismatic joint
    # lift: boolean, TRUE for lift, FALSE for place
    # extended: boolean, TRUE if extended, FALSE if retracted
    # IMPORTANT: To PLACE, rather than lift, reverse center_pitch and end_pitch
    def execute_lift_place(self, center_pitch, end_pitch, center_prism, extended, lift):
        for t in range(1,self.lift_res+1):
          if lift:
            c = self.clearance*t/self.lift_res
          else:
            c = self.clearance*(self.lift_res-t)/self.lift_res
          if extended:
            dist = self.x_dist
          else:
            dist = self.prism_len
          prism,ang = get_prism_ang(dist, c)
          self.dsb_joints[center_pitch] = ang
          self.dsb_joints[end_pitch] = -ang
          self.dsb_joints[center_prism] = (prism-self.prism_len)

          self.updateJointState() 
      
      
    # Executes an extend or retract operation
    # center_pitch: the slider widget for the appropriate center segment pitch joint
    # end_pitch: '' end segment pitch joint
    # center_prism: '' prismatic joint
    # extend: boolean, TRUE for extending, FALSE for retracting
    def execute_extend_retract(self, center_pitch, end_pitch, center_prism, extend):
        for t in range(1,self.xtend_res+1):
          if extend:
            nlen = self.prism_len + (self.x_dist-self.prism_len)*t/self.xtend_res
          else:
            nlen = self.prism_len + (self.x_dist-self.prism_len)*(self.xtend_res - t)/self.xtend_res
            
          prism, ang = get_prism_ang(nlen,self.clearance)
          self.dsb_joints[center_pitch] = ang
          self.dsb_joints[end_pitch] = -ang
          self.dsb_joints[center_prism] = (prism-self.prism_len)
          
          self.updateJointState()      
      
      
    # Executes a lift of the middle section
    # facing: 'fixed', 'forward', or 'backward'
    #   fixed: both ends retracted
    #   forward: front end extended
    #   backward: rear end extended
    # lift: lifting or placing, lift = True, place = False
    def execute_middle_lift_place(self, facing, lift):
        if facing == 'closed':
          xlen = self.prism_len
          xlen2 = self.prism_len
        elif facing == 'open':
          xlen = self.x_dist
          xlen2 = self.x_dist
        elif facing == 'forward':
          xlen = self.x_dist
          xlen2 = self.prism_len
        elif facing == 'backward':
          xlen = self.prism_len
          xlen2 = self.x_dist
        for t in range(1,self.lift_res+1):
        
          c = (self.clearance*t/self.lift_res) if lift else (self.clearance*(self.lift_res-t)/self.lift_res)

          prism, ang = get_prism_ang(xlen,c)          
          self.dsb_joints[CENTER_PITCHF] = -ang
          self.dsb_joints[FRONT_PITCH] = ang
          self.dsb_joints[CENTER_PRISMF] = prism - self.prism_len
          
          prism, ang = get_prism_ang(xlen2,c)
          self.dsb_joints[CENTER_PITCHR] = -ang
          self.dsb_joints[REAR_PITCH] = ang
          self.dsb_joints[CENTER_PRISMR] = prism - self.prism_len             
          
          self.updateJointState()  

      
    def execute_middle_extend_retract(self, facing):
        c = self.clearance
        for t in range(1, self.xtend_res+1):
          if facing == 'forward':
            flen = self.prism_len + (self.x_dist-self.prism_len)*(self.xtend_res - t)/self.xtend_res
            blen = self.prism_len + (self.x_dist-self.prism_len)*t/self.xtend_res
          else:
            flen = self.prism_len + (self.x_dist-self.prism_len)*t/self.xtend_res
            blen = self.prism_len + (self.x_dist-self.prism_len)*(self.xtend_res - t)/self.xtend_res
          
          prism, ang = get_prism_ang(flen,c)          
          self.dsb_joints[CENTER_PITCHF] = -ang
          self.dsb_joints[FRONT_PITCH] = ang
          self.dsb_joints[CENTER_PRISMF] = prism - self.prism_len
          
          prism, ang = get_prism_ang(blen,c)
          self.dsb_joints[CENTER_PITCHR] = -ang
          self.dsb_joints[REAR_PITCH] = ang
          self.dsb_joints[CENTER_PRISMR] = prism - self.prism_len   
          
          self.updateJointState()
      
    def execute_turn_straighten(self, angle, turn):
      for t in range(1, self.turn_res + 1):
        if turn:
          ang = angle*t/self.turn_res
        else:
          ang = angle*(self.turn_res-t)/self.turn_res
        self.dsb_joints[CENTER_SWIVEL] = ang
        self.updateJointState()
      
print("Gait Generator Initializing...")
gaitgen = GaitGenerator()
