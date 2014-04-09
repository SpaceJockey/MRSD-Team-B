#!/usr/bin/env python
import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import roslib
import sys
import rospy
from std_msgs.msg import String 

class SpaceJockeyCam(object):
    def __init__(self):
        self.Kmatrix_sub = rospy.Subscriber("spacecam/camera_info",CameraInfo,self.callback_Kmatrix)

    def callback_Kmatrix(self,data):
        self.Kmatrix = data.K
        
if __name__ == '__main__':
  spacecam_listener = SpaceJockeyCam()
  rospy.init_node('Kmatrix_listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"