#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import spacejockey

arduino = spacejockey.config("/arduino")
joints = spacejockey.config("/joints")

posArray = Int16MultiArray()  
posArray.data = [0, ] * len(arduino.joint_wiring)

def joints_cb(msg):
    for i in range(len(msg.name)):
        name = msg.name[i]
        pos = msg.position[i]
        #TODO: Use URDF min/max specs...
        rmin = joints.min[joints.name.index(name)]
        rmax = joints.max[joints.name.index(name)]
        posArray.data[arduino.joint_wiring.index(name)] = int((((pos - rmin) * (arduino.servo.max - arduino.servo.min)) / (rmax-rmin)) + arduino.servo.min)
            
    serialPub.publish(posArray)

if __name__ == '__main__':
    rospy.init_node('control_bridge')
    serialPub = rospy.Publisher('joint_ctl',Int16MultiArray)
    rospy.Subscriber('joint_states', JointState, joints_cb)
    rospy.spin()