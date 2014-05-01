#!/usr/bin/env python
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
import rospy
from cv_bridge import CvBridge, CvBridgeError 
import spacejockey
from spacejockey.srv import RobotStatus
import tf
import os,sys


class WorldWarp(object):
    def __init__(self):
        rospy.wait_for_service('robot_status') 
        try:
            self.get_status = rospy.ServiceProxy('robot_status', RobotStatus)
        except Exception, e:
            rospy.logerr("Service setup failed: " + str(e))
            sys.exit(-1)

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

        #using an image for this so we can specify complex patterns if desired
        #I.E. if we want to weight nearby (higher res) data higher or somesuch
        #Or even weight specific color bands higher
        self.vignette = np.float32(cv2.imread(rospy.get_param('/vignette_img'))) / 255.0
        self.image = None
        self.mask = None #alpha channel composite of vignettes

    def get_weight(self):
        status = self.get_status(rospy.Time.now())
        return status.image_weight

    def callback(self,data):
        try:
            camera_img = self.bridge.imgmsg_to_cv2(data, "passthrough")
            (trans, rot) = self.tfList.lookupTransform('/camera', '/world', rospy.Time(0))
            h = tf.TransformerROS().fromTranslationRotation(trans, rot)
        except Exception as e:
            rospy.logwarn(str(e))
            return

        height, width, depth = camera_img.shape
        if camera_img.size != self.vignette.size: #resize vignette to match camera data
            rospy.loginfo('resizing vignette mask to: ' + str((width, height)))
            self.vignette = cv2.resize(self.vignette, (width, height))

        camera_img = cv2.flip(camera_img, -1) #flip around both axes
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

        #warp the images to the world frame
        camera_warp = cv2.warpPerspective(camera_img,b,(self.width,self.height))
        vignette_warp = cv2.warpPerspective(self.vignette,b,(self.width,self.height))

        #combine the images
        a_new = self.get_weight() # weight for the pre image
        a_cur = 1.0 - a_new
        cMask = vignette_warp * a_new           #camera mask

        if self.image != None:
            #Get overlap mask
            wMask = np.subtract(np.ones(self.image.shape), cMask)     #world mask
            fg = np.uint8(np.multiply(camera_warp, cMask))
            bg = np.uint8(np.multiply(self.image, wMask))
            self.image = cv2.add(fg, bg)
            self.mask = np.add(cMask, self.mask)
        else:
            self.image = np.uint8(np.multiply(camera_warp, cMask))
            self.mask = cMask

    def publish_image(self):
        if self.image != None:
            try:
                #merge mask into image type
                b, g, r = cv2.split(self.image)
                a = np.uint8(cv2.split(self.mask)[0] * 255.0)
                rgba = cv2.merge([r, g, b, a])
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(rgba, "rgba8"))
            except Exception as e:
                rospy.logwarn(str(e))

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
