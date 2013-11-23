import roslib
import os
import rospkg
import rospy
import time

from math import pi, radians, degrees, cos, sin, tan, sqrt

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QMainWindow

# Given the length (dist) and height (clear) of a triangle,
# find the hypotenuse length (prism) and 
# angle (ang) between the length and hypotenuse    
def get_prism_ang(dist, clear):
  prism = sqrt(dist*dist + clear*clear)
  ang = degrees(tan(clear/dist))
  return prism, ang

class RqtSJWidget(QMainWindow):
    prism_len = 0.15 # Length of the constant portion of the prismatic joints
    
    # Set up Publishers and Subscribers
    pub = rospy.Publisher('joint_states',JointState)
    gaitPub = rospy.Publisher('gait_commands', String)
    schedPub = rospy.Publisher('sched_commands',String)
    
    # Set up constants for JointState messages for rViz
    dsb_joints = [0.0,]*12
    joint_msg = JointState()
    joint_msg.name = ['center_swivel','fore_base_pitch','aft_base_pitch','front_pitch','rear_pitch','fore_extend','aft_extend']
    joint_msg.position = dsb_joints[0:7]
    joint_msg.velocity=[0.0,]*7
    
    halted = False
    
    # Robot State
    extendedf = False
    extendedr = False
    liftedf = False
    liftedr = False
    liftedm = False
        
    
    def __init__(self, context):
    
        # GUI Setup
        super(RqtSJWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_sj'), 'resource', 'rqt_sj.ui')
        loadUi(ui_file, self)
        self.setObjectName('RqtSJWidget')
        
        # Initialize scheduler and gait parameters from GUI defaults
        self.clearance = self.clearance_dsb.value()        
        self.max_xtend = self.max_x_dsb.value()
        self.lift_res = self.liftplace_res_sb.value()
        self.xtend_res = self.x_res_sb.value()
        self.delay = self.delay_dsb.value()
        
        # Publish schedule and gait parameters to scheduler and gait_generator on startup
        self._handle_clearance_change(self.clearance_dsb.value())
        self._handle_max_extend_change(self.max_x_dsb.value())
        self._handle_lift_res_change(self.liftplace_res_sb.value())
        self._handle_x_res_change(self.x_res_sb.value())
        self._handle_delay_change(self.delay_dsb.value())
        
        
        # Wiring up command buttons
        self.queue_command.clicked[bool].connect(self._handle_queue_clicked) 
        self.zero_config.clicked[bool].connect(self._handle_zero_config)
        self.halt_all.clicked[bool].connect(self._handle_halt_all)
        self.clear_queue.clicked[bool].connect(self._handle_clear_queue)
        
        # Wiring up the buttons for lift/extend/place
        self.front_lift_button.clicked[bool].connect(self._handle_front_lift_clicked)
        self.front_extend_button.clicked[bool].connect(self._handle_front_extend_clicked)
        self.front_place_button.clicked[bool].connect(self._handle_front_place_clicked)
        self.middle_lift_button.clicked[bool].connect(self._handle_middle_lift_clicked)
        self.middle_extend_button.clicked[bool].connect(self._handle_middle_extend_clicked)
        self.middle_place_button.clicked[bool].connect(self._handle_middle_place_clicked)
        self.rear_lift_button.clicked[bool].connect(self._handle_rear_lift_clicked)
        self.rear_extend_button.clicked[bool].connect(self._handle_rear_extend_clicked)
        self.rear_place_button.clicked[bool].connect(self._handle_rear_place_clicked)
        
        # Initial Button Setup to Restrict Permissible Motions
        self.front_extend_button.setEnabled(False)
        self.front_place_button.setEnabled(False)
        self.rear_extend_button.setEnabled(False)
        self.rear_extend_button.setText('Extend')
        self.rear_place_button.setEnabled(False)   
        self.middle_extend_button.setEnabled(False)
        self.middle_place_button.setEnabled(False)     
        
        # Wiring up spinboxes for operational parameters
        self.clearance_dsb.valueChanged.connect(self._handle_clearance_change)
        self.max_x_dsb.valueChanged.connect(self._handle_max_extend_change)
        self.liftplace_res_sb.valueChanged.connect(self._handle_lift_res_change)
        self.x_res_sb.valueChanged.connect(self._handle_x_res_change)
        self.delay_dsb.valueChanged.connect(self._handle_delay_change)
        
        # Wiring up sliders for joint values
        self.center_swivel_sld.valueChanged.connect(self._handle_j0_change)
        self.center_pitchf_sld.valueChanged.connect(self._handle_j1_change)
        self.center_pitchr_sld.valueChanged.connect(self._handle_j2_change)
        self.front_pitch_sld.valueChanged.connect(self._handle_j3_change)
        self.rear_pitch_sld.valueChanged.connect(self._handle_j4_change)
        self.center_prismf_sld.valueChanged.connect(self._handle_j5_change)
        self.center_prismr_sld.valueChanged.connect(self._handle_j6_change)
        self.foot_center_check.stateChanged.connect(self._handle_j7_change)
        self.foot_front_left_check.stateChanged.connect(self._handle_j8_change)
        self.foot_front_right_check.stateChanged.connect(self._handle_j9_change)
        self.foot_rear_left_check.stateChanged.connect(self._handle_j10_change)
        self.foot_rear_right_check.stateChanged.connect(self._handle_j11_change)

        self.current_radio.setChecked(True)
        
        self.sub = rospy.Subscriber('joint_states', JointState, self.jointCallback)
        self.updateJointState()

    # Update all the gait segmet button states in the GUI
    def set_button_states(self):
        self.front_lift_button.setEnabled(not self.liftedf and not self.liftedr and not self.liftedm)
        self.middle_lift_button.setEnabled(not self.liftedf and not self.liftedr and not self.liftedm)
        self.rear_lift_button.setEnabled(not self.liftedf and not self.liftedr and not self.liftedm)
        self.front_place_button.setEnabled(self.liftedf)
        self.middle_place_button.setEnabled(self.liftedm)
        self.rear_place_button.setEnabled(self.liftedr)
        self.front_extend_button.setEnabled(self.liftedf)
        self.middle_extend_button.setEnabled(self.liftedm and ((self.extendedf and not self.extendedr)or(self.extendedr and not self.extendedf)))
        self.rear_extend_button.setEnabled(self.liftedr)
        
        self.front_extend_button.setText("Extend" if not self.extendedf else "Retract")
        self.rear_extend_button.setText("Extend" if not self.extendedr else "Retract")
        self.middle_extend_button.setText(self.get_middle_movement())
        
        
    # Determine State of Middle Segment
    # Forward: Poised to move forward
    # Backward: Poised to move backward
    # Open: Both end segments extended
    # Closed: Both end segments retracted
    def get_middle_movement(self):
        if self.extendedf and not self.extendedr:
          return "Forward"
        elif self.extendedr and not self.extendedf:
          return "Backward"
        elif self.extendedr and self.extendedf:
          return "Open"
        elif not self.extendedr and not self.extendedf:
          return "Closed"

    ########################################
    ########## UPDATING READOUTS ###########
    ########################################
    def jointCallback(self, msg):
        self.center_swivel_lbl.setText("%.2f" % degrees(msg.position[0]))
        self.center_fpitch_lbl.setText("%.2f" % degrees(msg.position[1]))
        self.center_rpitch_lbl.setText("%.2f" % degrees(msg.position[2]))
        self.front_pitch_lbl.setText("%.2f" % degrees(msg.position[3]))
        self.rear_pitch_lbl.setText("%.2f" % degrees(msg.position[4]))
        self.center_fprism_lbl.setText("%.2f" % msg.position[5]*100)
        self.center_rprism_lbl.setText("%.2f" % msg.position[6]*100)


    ########################################
    ########### COMMAND BUTTONS ############
    ########################################

    def _handle_queue_clicked(self, checked):
        self.updateJointState()
        
    def _handle_zero_config(self):
        self.center_swivel_sld.setValue(0)
        self.center_pitchf_sld.setValue(0)
        self.center_pitchr_sld.setValue(0)
        self.front_pitch_sld.setValue(0)
        self.rear_pitch_sld.setValue(0)
        self.center_prismf_sld.setValue(0)
        self.center_prismr_sld.setValue(0)
        time.sleep(0.1)
        self.updateJointState()       
        
    def _handle_halt_all(self, checked):
        if not self.halted:
          msg = String()
          msg.data = "pause"
          self.schedPub.publish(msg)
          self.halt_all.setText("Resume")
          self.halted = True
        else:
          msg = String()
          msg.data = "resume"
          self.schedPub.publish(msg)    
          self.halt_all.setText("HALT ALL")
          self.halted = False   
          
    def _handle_clear_queue(self, checked):
          msg = String()
          msg.data = "clear"
          self.schedPub.publish(msg)
      
    ########################################
    ########## OP. PARAM CHANGES ###########
    ########################################  

    def _handle_clearance_change(self, val):
        self.clearance = self.clearance_dsb.value()
        msg = String()
        msg.data = "clearance " + str(self.clearance)
        self.gaitPub.publish(msg)
    
    def _handle_max_extend_change(self, val):
        self.max_xtend = self.max_x_dsb.value()
        msg = String()
        msg.data = "maxextend " + str(self.max_xtend)
        self.gaitPub.publish(msg.data)         
    
    def _handle_lift_res_change(self, val):
        self.lift_res = self.liftplace_res_sb.value()
        msg = String()
        msg.data = "liftres " + str(self.lift_res)
        self.gaitPub.publish(msg.data)        
    
    def _handle_x_res_change(self, val):      
        self.xtend_res = self.x_res_sb.value()   
        msg = String()
        msg.data = "extendres " + str(self.xtend_res)
        self.gaitPub.publish(msg.data)         
        
    def _handle_delay_change(self,val):
        self.delay = self.delay_dsb.value()
        msg = String()
        msg.data = "delay " + str(self.delay)
        self.schedPub.publish(msg.data)


    ########################################
    ######## JOINT SLIDER CHANGES ##########
    ########################################

    def _handle_j0_change(self, val):
        self.dsb_joints[0]=radians(val)
        #self.updateJointState()
        
    def _handle_j1_change(self, val):
        self.dsb_joints[1]=radians(val)
        #self.updateJointState()

    def _handle_j2_change(self, val):
        self.dsb_joints[2]=radians(val)
        #self.updateJointState()
        
    def _handle_j3_change(self, val):
        self.dsb_joints[3]=radians(val)
        #self.updateJointState()

    def _handle_j4_change(self, val):
        self.dsb_joints[4]=radians(val)
        #self.updateJointState()

    def _handle_j5_change(self, val):
        self.dsb_joints[5]=val*0.001
        #self.updateJointState()

    def _handle_j6_change(self, val):
        self.dsb_joints[6]=val*0.001
        #self.updateJointState()

    def _handle_j7_change(self, val):
        self.dsb_joints[7]=val+0.0
        #self.updateJointState()

    def _handle_j8_change(self, val):
        self.dsb_joints[8]=val+0.0
        #self.updateJointState()

    def _handle_j9_change(self, val):
        self.dsb_joints[9]=val+0.0
        #self.updateJointState()

    def _handle_j10_change(self, val):
        self.dsb_joints[10]=val+0.0
        #self.updateJointState()

    def _handle_j11_change(self, val):
        self.dsb_joints[11]=val+0.0
        #self.updateJointState()
        
    # Publish a joint state to joint_states (for Rviz to display, usually)
    def updateJointState(self):
        if self.current_radio.isChecked():
          self.joint_msg.header.stamp = rospy.Time.now()
          self.joint_msg.position = self.dsb_joints[0:7]
          self.pub.publish(self.joint_msg)

    # Send a command to teh gait scheduler
    def gait_gen(self, command):
        print(command)
        msg = String()
        msg.data = command
        self.gaitPub.publish(msg)


    #### LIFT AND PLACE HANDLERS ####s   
    def _handle_front_lift_clicked(self):
        self.gait_gen("lift front " + ("extended" if self.extendedf else "retracted" ))
        self.liftedf = True
        self.set_button_states()
    
    def _handle_front_place_clicked(self):
        self.gait_gen("place front " + ("extended" if self.extendedf else "retracted") )
        self.liftedf = False
        self.set_button_states()
        
    def _handle_rear_lift_clicked(self):
        self.gait_gen("lift rear " + ("extended" if self.extendedr else "retracted")  )
        self.liftedr = True
        self.set_button_states()        
       
    def _handle_rear_place_clicked(self):
        self.gait_gen("place rear " + ("extended" if self.extendedr else "retracted")  )
        self.liftedr = False
        self.set_button_states()       

    #### EXTENSION HANDLERS ####
    def _handle_front_extend_clicked(self):
        if self.extendedf:
          self.gait_gen("retract front") 
          self.front_extend_button.setText('Extend')
        else:
          self.gait_gen("extend front") 
          self.front_extend_button.setText('Retract')        
        self.extendedf = not self.extendedf
        self.set_button_states()
        
    def _handle_rear_extend_clicked(self):
        if self.extendedr:
          self.gait_gen("retract rear") 
          self.rear_extend_button.setText('Extend')
        else:
          self.gait_gen("extend rear") 
          self.rear_extend_button.setText('Retract')        
        self.extendedr = not self.extendedr   
        self.set_button_states()     

    def _handle_middle_lift_clicked(self):
        self.gait_gen("lift middle " + self.get_middle_movement().lower())
        self.liftedm = True
        self.set_button_states()
        
    def _handle_middle_extend_clicked(self): 
        mid = self.get_middle_movement() 
        self.extendedr = not self.extendedr
        self.extendedf = not self.extendedf
        self.set_button_states() 
        #self.extend_middle()
        
    def _handle_middle_place_clicked(self):
        self.gait_gen("place middle " + self.get_middle_movement().lower())
        self.liftedm = False
        self.set_button_states()     


    def extend_middle(self):
        ang = 0
        ext = 0
        xlen = 0.15
        xlen2 = 0.3
        clear = 0.05
        dist = sqrt(xlen*xlen - clear*clear)
        for t in range(0,50):
          nlen = xlen + dist*(t/50.0)
          prism, ang = get_prism_ang(nlen,clear)
          self.center_pitchr_sld.setValue(-ang)
          self.rear_pitch_sld.setValue(ang)
          self.center_prismr_sld.setValue((prism-xlen)*1000)
          
          nlen = xlen + dist*((50-t)/50.0)
          prism, ang = get_prism_ang(nlen,clear)
          self.center_pitchf_sld.setValue(-ang)
          self.front_pitch_sld.setValue(ang)
          self.center_prismf_sld.setValue((prism-xlen)*1000)
          
          self.updateJointState()
          #time.sleep(self.delay)
          
          

    def shutdown_all(self):
        rospy.loginfo("Shutting down...")
