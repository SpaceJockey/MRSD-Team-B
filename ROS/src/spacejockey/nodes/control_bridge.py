#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import spacejockey

arduino = spacejockey.config("/arduino")
joints = spacejockey.urdf.joint_map

posArray = Int16MultiArray()  
posArray.data = [0, ] * len(arduino.joint_wiring)

def joints_cb(msg):
    for i in range(len(msg.name)):
        try:
            name = msg.name[i]
            pos = msg.position[i]
            joint = joints[name]
            #TODO: clip to servo mins/maxes
            posArray.data[arduino.joint_wiring.index(name)] = int((((pos - joint.limit.lower) * (arduino.servo.max - arduino.servo.min)) / (joint.limit.upper-joint.limit.lower)) + arduino.servo.min)
        except: #don't care about bad joint mappings...
            continue
            
    serialPub.publish(posArray)

if __name__ == '__main__':
    rospy.init_node('control_bridge')
    serialPub = rospy.Publisher('joint_ctl',Int16MultiArray)
    rospy.Subscriber('joint_states', JointState, joints_cb)
    rospy.spin()