#!/usr/bin/env python
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import rospy
from cv_bridge import CvBridge, CvBridgeError 
import spacejockey
import tf
import os,sys


class WorldWarp(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("dirty_map",Image)
        self.image_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)
        k = rospy.get_param("/camera_matrix/data")
        self.K=np.matrix([[k[0],k[1],k[2]],[k[3],k[4],k[5]],[k[6],k[7],k[8]]])
        self.tfList = tf.TransformListener() # for the getting big H
        self.width  = rospy.get_param("/gui/width")
        self.height = rospy.get_param("/gui/height")
        self.scale  = 1.0 / rospy.get_param("/gui/scale")
        self.origin = spacejockey.config("/gui/origin")
        self.image = None

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
            (trans, rot) = self.tfList.lookupTransform('/camera', '/world', rospy.Time(0))
            h = tf.TransformerROS().fromTranslationRotation(trans, rot)
        except Exception as e:
            rospy.logwarn(str(e))
            return

        height, width, depth = cv_image.shape
        ## GET first two columns of R and column of T to form matrix_RT
        ## Since Z=0, we can delete third column of R
        matrix_RT=np.matrix([[h[0,0],h[0,1],h[0,3]],[h[1,0],h[1,1],h[1,3]],[h[2,0],h[2,1],h[2,3]]])
        a = self.K * matrix_RT

        # image point height= 480=u, width=640=v, using (u,v,1)
        # camera information 480*640
        # scale the real world to the worldImage 
        c = np.matrix([[self.scale,0,self.origin.x],[0,-self.scale,self.origin.y],[0,0,1]])
        # inverse the 3*3 matrix
        b = c*a.I 

        dst = cv2.warpPerspective(cv_image,b,(self.width,self.height))
        if self.image != None:
            a_new = 0.5 # weight for the pre image
            a_cur = 1.0 - a_new   # weight for the current image
            # compare the temp pixel values and dst pixel values

            newGray = cv2.cvtColor(dst,cv2.COLOR_BGR2GRAY)
            oldGray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            ret, newMask = cv2.threshold(newGray, 10, 255, cv2.THRESH_BINARY)
            ret, oldMask = cv2.threshold(oldGray, 10, 255, cv2.THRESH_BINARY)
            mask = cv2.multiply(oldMask, newMask)
            mask_inv = cv2.bitwise_not(mask)

            bg = cv2.add(self.image, dst, mask = mask_inv) # cv2.bitwise_and(dst, dst, mask = cv2.bitwise_not(oldMask))
            fg = cv2.addWeighted(self.image, a_cur, dst, a_new, 0)
            blank = np.zeros(fg.shape, np.uint8)
            fg = cv2.add(fg, blank, mask = mask)
            self.image = cv2.add(fg, bg)
        else:
            self.image=dst

    def publish_image(self):
        if self.image != None:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "passthrough"))
            except Exception as e:
                rospy.logwarn(str(e))

    #TODO: serverize this!
    #TODO: perameterize the file name
    def write_image(self):
        ### this is save the World_Image for the final baseline 
        cv2.imwrite(os.path.dirname(sys.argv[0])+"/../config/clean_map.png",self.image)

if __name__ == '__main__':
    rospy.init_node('world_warp', anonymous=True)
    world_warp = WorldWarp()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            world_warp.publish_image()
            rate.sleep()
        except:
            break
    #world_warp.write_image()
