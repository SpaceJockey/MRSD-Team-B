#!/usr/bin/env python
import rospy, time
from std_msgs.msg import String, Empty, Float32MultiArray
from sensor_msgs.msg import JointState

    
class Scheduler():
    delay = 1       # Default delay between queue pops
    pause = False   # Not-paused by default
    queue = []      # Joint configuration queue
    posArray = Float32MultiArray()    

    # Pubisher for joint states
    jointPub = rospy.Publisher('joint_states',JointState)
    serialPub = rospy.Publisher('joint_ctl',Float32MultiArray)
    
    # Init: Set up listener
    def __init__(self):
      print("Starting up scheduler...")
      rospy.init_node('scheduler', anonymous=True)
      self.listener()
      print("Scheduler online.")
      self.print_queue_len()
            
    # Listens for commands and joint configurations to add to the queue
    def listener(self):
      rospy.Subscriber("sched_joints", JointState, self.addto_scheduler)
      rospy.Subscriber("sched_commands", String, self.command_scheduler)   
      
    # Adds a joint configuration to the queue
    def addto_scheduler(self,msg):
      self.queue.insert(0,msg)
      print("Joint State pushed.")
      
    def print_queue_len(self):
      print("Queue Size: " + str(len(self.queue)))
      
    # Receives commands from the main GUI
    def command_scheduler(self,msg):
      print("Scheduler: " + msg.data)
      if(msg.data == "pause"):
        self.pause = True
      elif(msg.data == "resume"):
        self.pause = False
        sched.print_queue_len()
      elif(msg.data == "clear"):
        self.queue = []
      elif(msg.data.split()[0]=="delay"):
        self.delay = float(msg.data.split()[1])
      else:
        print("Invalid command!")
 
sched = Scheduler()

# Scheduler loop. While not paused, continually print queue size every (delay) seconds.
# If queue larger than zero, pop a joint configuration each cycle
#   and publish it to joint_states to be received by rViz and/or the orbot
while not rospy.is_shutdown():
  time.sleep(sched.delay)
  if (not sched.pause):
    if (len(sched.queue) > 0):
      comm = sched.queue.pop()
      sched.jointPub.publish(comm)
      sched.posArray.data = comm.position
      sched.serialPub.publish(sched.posArray)
      sched.print_queue_len()
