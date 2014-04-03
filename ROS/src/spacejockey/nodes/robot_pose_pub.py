#!/usr/bin/env python  
#mostly copied from ROS joint_state_publisher
import rospy
import xml.dom.minidom
from sensor_msgs.msg import JointState
from math import pi
import tf
import IPython

RANGE = 10000

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class RobotInitPublisher():
    def __init__(self):
        description = get_param('robot_description')
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})

        self.zeros = get_param("zeros")

        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval}
                joint['position'] = zeroval    
                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint

    def msg(self):
        # Publish Joint States
        msg = JointState()
        msg.header.stamp = rospy.Time.now()

        # Initialize msg.position
        num_joints = (len(self.free_joints.items()) +
                      len(self.dependent_joints.items()))
        msg.position = num_joints * [0.0]

        for i, name in enumerate(self.joint_list):
            msg.name.append(str(name))
            joint = None

            # Add Free Joint
            if name in self.free_joints:
                joint = self.free_joints[name]
            
            msg.position[i] = joint['position']
        return msg

pub = rospy.Publisher('joint_states', JointState)
rospy.init_node('robot_init', anonymous=True)
tfCast = tf.TransformBroadcaster()
jsp = RobotInitPublisher()
pub.publish(jsp.msg())  
tfCast.sendTransform((0,0,0), tf.transformations.quaternion_from_euler(1.57, 0, 0), rospy.Time.now(), "center_foot", "world")
#IPython.embed()
rospy.spin()      
