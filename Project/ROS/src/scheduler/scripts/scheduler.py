#!/usr/bin/env python
import rospy, time
from std_msgs.msg import String, Empty, Float64MultiArray
from sensor_msgs.msg import JointState

    
class Scheduler():
    delay = 1       # Default delay between queue pops
    pause = False   # Not-paused by default
    queue = []      # Joint configuration queue
    
    # Pubisher for joint states
    jointPub = rospy.Publisher('joint_states',JointState)
    serialPub = rospy.Publisher('serial_link',Float64MultiArray)
    #emptyPub = rospy.Publisher('toggle_led',Empty)
    
    # Init: Set up listener
    def __init__(self):
      rospy.init_node('scheduler', anonymous=True)
      self.listener()
      print("Scheduler online.")
      
    # Listens for commands and joint configurations to add to the queue
    def listener(self):
      rospy.Subscriber("sched_joints", JointState, self.addto_scheduler)
      rospy.Subscriber("sched_commands", String, self.command_scheduler)   
      
    # Adds a joint configuration to the queue
    def addto_scheduler(self,msg):
      print("Scheduler received a joint state!")
      self.queue.insert(0,msg)
      
    # Receives commands from the main GUI
    def command_scheduler(self,msg):
      print("Scheduler: " + msg.data)
      if(msg.data == "pause"):
        self.pause = True
      elif(msg.data == "resume"):
        self.pause = False
      elif(msg.data == "clear"):
        self.queue = []
      elif(msg.data.split()[0]=="delay"):
        self.delay = float(msg.data.split()[1])
 

print("Starting up scheduler...")
sched = Scheduler()

# Scheduler loop. While not paused, continually print queue size every (delay) seconds.
# If queue larger than zero, pop a joint configuration each cycle
#   and publish it to joint_states to be received by rViz and/or the orbot
while (True):
  time.sleep(sched.delay)
  if (not sched.pause):
    if (len(sched.queue) > 0):
      comm = sched.queue.pop()
      sched.jointPub.publish(comm)
      posArray = Float64MultiArray()
      posArray.data = comm.position
      sched.serialPub.publish(posArray)
    print("Queue Size: " + str(len(sched.queue)))
