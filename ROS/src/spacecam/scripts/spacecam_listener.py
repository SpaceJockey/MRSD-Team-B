#!/usr/bin/env python
import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import roslib
import sys
import rospy
import cv
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 

class SpaceJockeyCam(object):
    def __init__(self):
        #cv.NamedWindow("Space Cam Listener", 1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("spacecam/image_raw",Image,self.callback)
        self.Kmatrix_sub = rospy.Subscriber("spacecam/camera_info",CameraInfo,self.callback_Kmatrix)


    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError, e:
            print e

        (cols,rows) = cv.GetSize(cv_image)
        if cols > 60 and rows > 60 :
            cv.Circle(cv_image, (50,50), 10, 255)

        cv.ShowImage("SpaceCam Listener Image", cv_image)
        cv.WaitKey(3)

    def callback_Kmatrix(self,data):
        self.Kmatrix = data.K
        
if __name__ == '__main__':
  spacecam_listener = SpaceJockeyCam()
  rospy.init_node('spacecam_listener', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()