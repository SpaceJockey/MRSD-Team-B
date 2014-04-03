#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import spacejockey

arduino = spacejockey.config("/arduino")

posArray = Float32MultiArray()  
posArray.data = [0.0, ]*10

def joints_cb(msg):
    for j in arduino.ctl_msg_order:
        if msg.position:
            index = msg.name.index(j)
            posArray.data[arduino.ctl_msg_order.index(j)] = msg.position[index]
        else:
            position = None
    serialPub.publish(posArray)

if __name__ == '__main__':
    rospy.init_node('control_bridge')
     # Pubisher for joint states
    serialPub = rospy.Publisher('joint_ctl',Float32MultiArray)
    rospy.Subscriber('joint_states', JointState, joints_cb)

    rospy.spin()