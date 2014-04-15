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
import tf
import geometry_msgs.msg
import copy

class CV_tf_cvimages(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)
        k = rospy.get_param("/camera_matrix/data")
        self.K=np.matrix([[k[0],k[1],k[2]],[k[3],k[4],k[5]],[k[6],k[7],k[8]]])
        self.tfList = tf.TransformListener() # for the getting big H

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            self.tfList.waitForTransform('/camera', '/world', data.header.stamp, rospy.Duration(0.1))
            (trans, rot) = self.tfList.lookupTransform('/camera', '/world', data.header.stamp)
            h = tf.TransformerROS().fromTranslationRotation(trans, rot)
        except Exception as e:
            rospy.logerr(str(e))
            return
        
        # print h

        # print cv_image.shape[0]
        # print cv_image.shape[1]
        # print cv_image[0,0]
        # print cv_image[479,639]

        height, width, depth = cv_image.shape
        gray1=cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
        #print "self.K",self.K
        ## GET first two colomuns of R and colomun of T
        ## to form matrxi_RT
        ## Since Z=0, we can delete third colomun of R
        matrix_RT=np.matrix([[h[0,0],h[0,1],h[0,3]],[h[1,0],h[1,1],h[1,3]],[h[2,0],h[2,1],h[2,3]]])
        print matrix_RT
        a = self.K * matrix_RT

        # image point height= 480=u, width=640=v
        # using (u,v,1)
        # camera information 480*640
        # 3.2 inches x 2.4 inches, or 8.1 x 6.1 cm
        # here assume our test surface to be 3*3 meters
        # here set the world map to be 600*600
        
        worldReal_width = 3
        worldReal_height = 3
        worldImage_width = 600
        worldImage_height = 600
        width_scale = worldImage_width / worldReal_width
        height_scale = worldImage_height / worldReal_height
        # scale the real world to the worldImage 
        c = np.matrix([[height_scale,0,0],[0,width_scale,0],[0,0,1]])
        # inverse the 3*3 matrix
        b = c*a.I 
        
        ##image points
        left_bottom_Image=np.matrix([[0],[0],[1]])
        right_bottom_Image=np.matrix([[0],[640],[1]])
        right_top_Image=np.matrix([[480],[640],[1]])
        left_top_Image=np.matrix([[480],[0],[1]])

        ## world points
        left_bottom_World=b*left_bottom_Image
        right_bottom_World=b*right_bottom_Image
        right_top_World=b*right_top_Image
        left_top_World=b*left_top_Image
        W_LB=left_bottom_World/left_bottom_World[2,0]
        W_RB=right_bottom_World/right_bottom_World[2,0]
        W_RT=right_top_World/right_top_World[2,0]
        W_LT=left_top_World/left_top_World[2,0]
        x1=(W_LB[0,0])
        y1=(W_LB[1,0])
        x2=(W_RB[0,0])
        y2=(W_RB[1,0])
        x3=(W_RT[0,0])
        y3=(W_RT[1,0])
        x4=(W_LT[0,0])
        y4=(W_LT[1,0])

        print " warped four points"
        print x1,y1
        print x2,y2
        print x3,y3
        print x4,y4

        ## TODO
        ## need to overlap all the images from camera  to a big worldImage
        ## need to store this worldImage to ROS images msgs for future calling for image comparison part
        dst = cv2.warpPerspective(cv_image,b,(worldImage_width,worldImage_height))
        cv2.imshow("World_Image", dst)

        cv2.waitKey(3)


 
if __name__ == '__main__':
  rospy.init_node('CV_listener', anonymous=True)
  CV_listener = CV_tf_cvimages()
  rospy.spin() 
  cv2.destroyAllWindows()
