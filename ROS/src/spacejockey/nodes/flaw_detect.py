#!/usr/bin/env python
####   Inspection for spacejockey
####   Songjie Zhong
####   03/19/2014
####   update date 04/22/2014
####   add k-means for the bonding box and center informations
####   compare the difference of the two world images for the independent testing
####   integrated in ROS

import numpy as np
import cv, cv2
import os,sys
from sensor_msgs.msg import Image
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from spacejockey import config

# for the marker part
from visualization_msgs.msg import Marker

class FlawDetector(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.defect_pub = rospy.Publisher('/visualization_marker', Marker)

        #cache K value, should never find less defects than previous iteration...
        self.K = 1

        self.config = config('/flaws')
        self.gui = config('/gui')

        self.ksize = tuple(self.config.filter_size)
        self.sigma = self.config.filter_sigma
        self.threshold = self.config.detect_threshold
        self.max_size = self.config.detect_max_size / self.gui.scale

        #test data
        #dirty = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_dirty.png") 
        #clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_clean.png") 
        #
        ##preprocess dirty image
        #self.dirtyImage = cv2.GaussianBlur(dirty, self.ksize, self.sigma)
        #self.dirtyHue = cv2.split(cv2.cvtColor(self.dirtyImage, cv2.COLOR_BGR2HSV))[0]

        #real data
        #TODO: perameterize config file name
        clean = cv2.imread(rospy.get_param('/clean_env_map'))
        # self.dirty = np.zeros(self.clean.shape, dtype=np.uint8)
        
        #preprocess clean image
        self.cleanImage = cv2.GaussianBlur(clean, self.ksize, self.sigma)
        self.cleanHue = cv2.split(cv2.cvtColor(self.cleanImage, cv2.COLOR_BGR2HSV))[0]

        #init other stuff...
        self.dirtyImage = None
        self.dirtyHue = None
        self.alphaMask = None
        self.shape = clean.shape
        self.size = clean.size
        self.updated = True


    def callback(self,data):
        try:
            dirty = cv2.split(self.bridge.imgmsg_to_cv2(data, "passthrough"))
            alpha = dirty[3]
            foobar, self.alphaMask = cv2.threshold(alpha, 128, 255, cv.CV_THRESH_BINARY) #alpha channel
            dirty = cv2.merge([dirty[2], dirty[1], dirty[0]])
            assert dirty.size == self.size, 'Image size mismatch:' + str(dirty.size) + " != " + str(self.size)
        except Exception as e:
            rospy.logerr(str(e))
            return
        
        #preprocess dirty image
        self.dirtyImage = cv2.GaussianBlur(dirty, self.ksize, self.sigma)
        self.dirtyHue = cv2.split(cv2.cvtColor(self.dirtyImage, cv2.COLOR_BGR2HSV))[0]
        self.updated = False

    def kMeans(self, K, Z):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1) #TODO: test different iteration counts...
        ret, label, center = cv2.kmeans(Z, K, criteria, 100, cv2.KMEANS_RANDOM_CENTERS)

        bboxes = []
        maxDim = 0
        for j in range(K):
            A = Z[label.ravel() == j]
            
            width = int(A[:,0].max()) - int(A[:,0].min())
            if (width > maxDim):
                maxDim = width

            height = int(A[:,1].max()) - int(A[:,1].min())
            if (height > maxDim):
                maxDim = height
            bboxes.append((center[j], (width,height), j))
        return (maxDim, bboxes)



    def update(self):
        rospy.logdebug('Updating')
        if self.updated:
            return
        self.updated = True
        
        diff = cv2.absdiff(self.dirtyHue, self.cleanHue)
        diff = cv2.bitwise_and(diff, diff, mask = self.alphaMask)
        
        Z = []
        for x in range(self.shape[0]):
            for y in range(self.shape[1]):
                if rospy.is_shutdown():
                    sys.exit(0)          
                if diff[x, y] > self.threshold:
                    Z.append([x, y])

        if not len(Z):
            rospy.loginfo('Clean surface, no defects found')
            return
        rospy.loginfo('Defects found, running K-Means...')

        #matrix cast for numpy
        Z = np.matrix(Z, dtype=np.float32)

        # how to determine k value by using k-means clustering method
        rospy.logdebug('Running K-Means')

        maxDim, bboxes = self.kMeans(self.K, Z)

        #make sure K groupings are small enough
        while maxDim > self.max_size and not rospy.is_shutdown():
            rospy.logdebug('Updating K value: ' + str(self.K))
            self.K += 1 
            maxDim, bboxes = self.kMeans(self.K, Z)

        rospy.loginfo('found ' + str(self.K) +' Defects.')
        #output markers
        rospy.logdebug('Dumping Markers')
        for b in bboxes:
            ######################################
            # for the markers part
            ######################################
            marker = Marker()
            marker.header.frame_id = "world"  
            marker.header.stamp = rospy.Time()
            marker.ns = "flaws"
            marker.id = b[2]
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.pose.position.x = float(b[0][1] - self.gui.origin.x) * self.gui.scale
            marker.pose.position.y = float(self.gui.origin.y - b[0][0]) * self.gui.scale
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(b[1][1]) * self.gui.scale
            marker.scale.y = float(b[1][0]) * self.gui.scale
            marker.scale.z = 0.01
            marker.color.a = 0.75
            marker.color.r = 1.0
          
            # print marker
            self.defect_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('flaw_detect')
    flaw_detect = FlawDetector()
    rospy.loginfo('Flaw Detection Online')
    worldImage_sub = rospy.Subscriber("dirty_map", Image, flaw_detect.callback)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        flaw_detect.update()
        rate.sleep()
    cv2.destroyAllWindows()



