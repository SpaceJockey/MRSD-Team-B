#!/usr/bin/env python
import cv2
import urllib 
import numpy as np
from sensor_msgs.msg import Image 
import roslib
import sys
import rospy
import cv
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import CameraInfo
import camera_info_manager
import argparse

class SpaceJockeyCam(object):
    def __init__(self, url):
        try:
            self.stream=urllib.urlopen(url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(url))
            sys.exit() #'Unable to open camera stream')
        self.bytes=''
        self.width = 640
        self.height = 480
        self.frame_id = 'camera'
        self.image_pub = rospy.Publisher("camera/image_raw", Image)
        self.cinfo = camera_info_manager.CameraInfoManager(cname = 'camera', url = 'PACKAGE://spacecam/config/calibration.yaml')
        self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.caminfo_pub = rospy.Publisher("camera/camera_info", CameraInfo)
        self.bridge = CvBridge()

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
        cimsg = self.cinfo.getCameraInfo()
        cimsg.header.stamp = rospy.Time.now()
        cimsg.header.frame_id = self.frame_id
        cimsg.width = self.width
        cimsg.height = self.height
        self.caminfo_pub.publish(cimsg)

    def publishMsg(self):
        '''Publish jpeg image as a ROS message'''
        self.msg = Image()
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = self.frame_id
        self.msg.data = self.img
        self.axis.pub.publish(self.msg)

def main():
    parser = argparse.ArgumentParser(prog='spacecam.py', description='reads a given url string and dumps it to a ros_image topic')
    parser.add_argument('-g', '--gui', action='store_true', help='show a GUI of the camera stream')
    parser.add_argument('-u', '--url', default='http://admin:admin@10.68.68.22/goform/video?channel=1&.mjpg', help='camera stream url to parse')
    args, unknown = parser.parse_known_args()
    
    rospy.init_node('SpaceJockeyCamera', anonymous=True)
    spacecam = SpaceJockeyCam(args.url)

    while not rospy.is_shutdown():
        spacecam.bytes += spacecam.stream.read(1024)
        a = spacecam.bytes.find('\xff\xd8')
        b = spacecam.bytes.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = spacecam.bytes[a:b+2]
            spacecam.bytes= spacecam.bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            image_message = spacecam.bridge.cv2_to_imgmsg(i, "bgr8")
            image_message.header.stamp = rospy.Time.now()
            image_message.header.frame_id = spacecam.frame_id
            spacecam.image_pub.publish(image_message)
            spacecam.publishCameraInfoMsg()
            print "height: ", image_message.height
            print "width: ", image_message.width
            print "encoding: ", image_message.encoding
            print "step: ", image_message.step
            print "header: ", image_message.header
            if args.gui:
                cv2.imshow('Space Jockey Publisher Cam',i)
            if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
                exit(0) 
    



if __name__ == '__main__':
    main()
