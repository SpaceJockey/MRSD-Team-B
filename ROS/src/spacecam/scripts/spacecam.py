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
import argparse

#todo: make some input parse arguments like, showing the GUI or not
try:
    cam_url = rospy.get_param("/spacecam/url")
except:
    cam_url = 'http://admin:admin@10.68.68.22/goform/video?channel=1&.mjpg'

class SpaceJockeyCam(object):
    def __init__(self):
        try:
            self.stream=urllib.urlopen(cam_url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(cam_url))
            sys.exit('Unable to open camera stream')
        self.bytes=''
        self.image_pub = rospy.Publisher("spacecam_image",Image)
        self.bridge = CvBridge()

if __name__ == '__main__':


    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-g', '--gui', type=str, default='no',
                        help='To show the GUI or not')
    args = parser.parse_args()

    if args.gui == 'no':
        visualize = False
    elif args.gui == 'yes':
        visualize = True
    else:
        print 'Unknown GUI option: %s' % args.gui
        exit(0)
    
    rospy.init_node('SpaceJockeyCamera', anonymous=True)
    spacecam = SpaceJockeyCam()

    while True:
        spacecam.bytes+=spacecam.stream.read(1024)
        a = spacecam.bytes.find('\xff\xd8')
        b = spacecam.bytes.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = spacecam.bytes[a:b+2]
            spacecam.bytes= spacecam.bytes[b+2:]
            i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            # image_message = spacecam.bridge.cv_to_imgmsg(i, encoding="passthrough")
            image_message = cv.fromarray(i)
            spacecam.image_pub.publish(spacecam.bridge.cv_to_imgmsg(image_message, "bgr8"))

            if visualize:
                cv2.imshow('Space Jockey Publisher Cam',i)
            if cv2.waitKey(1) ==27: # wait until ESC key is pressed in the GUI window to stop it
                exit(0) 
