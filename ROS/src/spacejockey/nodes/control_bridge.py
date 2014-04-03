#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

posArray = Float32MultiArray()  
posArray.data = [0.0, ]*9

def joints_cb(msg):
    #TODO: Double-check ordering to match arduino side...
    for i in range(9):
        if msg.position:
            posArray.data[i] = msg.position[i]
        else:
            position = None
    serialPub.publish(posArray)

if __name__ == '__main__':
    rospy.init_node('control_bridge')
     # Pubisher for joint states
    serialPub = rospy.Publisher('joint_ctl',Float32MultiArray)
    rospy.Subscriber('joint_states', JointState, joints_cb)

    rospy.spin()