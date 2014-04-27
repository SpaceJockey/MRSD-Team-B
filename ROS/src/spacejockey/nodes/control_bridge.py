#!/usr/bin/env python 
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Int16MultiArray, Float32
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
            servo = int((((pos - joint.limit.lower) * (arduino.servo.max - arduino.servo.min)) / (joint.limit.upper-joint.limit.lower)) + arduino.servo.min)
            if servo < arduino.servo.min:
                servo = arduino.servo.min
            if servo > arduino.servo.max:
                servo = arduino.servo.max 
            posArray.data[arduino.joint_wiring.index(name)] = servo
        except: #ignore missing joint mappings
            continue
    serialPub.publish(posArray)

def batt_cb(msg):
    #scale battery value to a 0-1 float scale
    fBatt = float(msg.data - arduino.battery.min) / (arduino.battery.max - arduino.battery.min)
    battPub.publish(fBatt)

if __name__ == '__main__':
    rospy.init_node('control_bridge')
    serialPub = rospy.Publisher('joint_ctl',Int16MultiArray)
    rospy.Subscriber('joint_states', JointState, joints_cb)

    battPub = rospy.Publisher('battery_state', Float32)
    rospy.Subscriber('battery_raw', Int16, batt_cb)

    rospy.spin()