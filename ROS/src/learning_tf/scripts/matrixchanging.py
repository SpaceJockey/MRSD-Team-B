#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic


import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String
import copy

def callback(data):

    str=copy.deepcopy(data.data)
    rospy.loginfo(data.data)

    #data.data
    # Id=7 (Hamming=0) distance=0.32224m x=0.307158 y=0.0930037 z=-0.0290389 yaw=1.64067 pitch=0.0604723 roll=-0.297036 done

    str_list=str.split(" ")
    # print str_list
    # ['Id=7', '(Hamming=0)', 'distance=0.32224m', 'x=0.307158', 'y=0.0930037', 'z=-0.0290389', 'yaw=1.64067', 'pitch=0.0604723', 'roll=-0.297036', 'done\n']

    m = geometry_msgs.msg.TransformStamped()
    m.transform.translation.x = float(str_list[3][2:])
    # print m.transform.translation.x
    m.transform.translation.y = float(str_list[4][2:])
    # print m.transform.translation.y
    m.transform.translation.z = float(str_list[5][2:])
    # print m.transform.translation.z
    
    yaw=float(str_list[6][4:])
    # print yaw
    pitch=float(str_list[7][7:])
    # print pitch
    roll=float(str_list[8][5:])
    # print roll

    ### tf.transformations.quaternion_from_euler(roll,pitch,yaw)
    ### this roll, pitch ,yaw angles can get from running apriltags.demo
    ### saved in localization.txt

    a=tf.transformations.quaternion_from_euler(roll,pitch,yaw)

    m.transform.rotation.x = a[0]
    m.transform.rotation.y = a[1]
    m.transform.rotation.z = a[2]
    m.transform.rotation.w = a[3]

    #print m
    pub_tf=rospy.Publisher("/tf", tf.msg.tfMessage)
    tfm = tf.msg.tfMessage([m])

    pub_tf.publish(tfm)


    translation=(m.transform.translation.x, m.transform.translation.y, m.transform.translation.z)
    rotation=(m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w)


    fourbyfourmatrix=tf.TransformerROS().fromTranslationRotation(translation,rotation)
    print fourbyfourmatrix  

    #rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('matrixchanging', anonymous=True)


   


    rospy.Subscriber("chatter", String, callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()










# Roll phi: rotation about the X-axis
# Pitch theta: rotation about the Y-axis
# Yaw psi: rotation about the Z-axis

### here is the already exited commands from the package transformations.py from tf

# tf.transformations.quaternion_about_axis
# tf.transformations.quaternion_conjugate
# tf.transformations.quaternion_from_euler
# tf.transformations.quaternion_from_matrix
# tf.transformations.quaternion_inverse
# tf.transformations.quaternion_matrix
# tf.transformations.quaternion_multiply
# tf.transformations.quaternion_slerp