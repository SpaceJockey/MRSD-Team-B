#!/usr/bin/env python
####   Inspection for spacejockey
####   Songjie Zhong
####   03/19/2014
####   update date 04/22/2014
####   add k-means for the bonding box and center informations
####   compare the difference of the two world images for the independent testing
####   integrated in ROS

import numpy as np
import cv2
import os,sys
from sensor_msgs.msg import Image
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError 

# for the marker part
from visualization_msgs.msg import Marker

class FlawDetector(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.defect_pub = rospy.Publisher('/visualization_marker', Marker)

        #cache K value, should never find less defects than previous iteration...
        self.K = 1

        #test data
        self.dirty = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_dirty.png") 
        self.clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_clean.png") 

        #real data
        #TODO: perameterize config file name
        # self.clean = cv2.imread(rospy.get_param('/clean_env_map'))
        # self.dirty = np.zeros(self.clean.shape, dtype=np.uint8)
        self.shape = self.clean.shape
        self.updated = False


    def callback(self,data):
        clean = self.clean
        try:
            dirty = self.bridge.imgmsg_to_cv2(data, "passthrough")
            assert dirty.shape == clean.shape, 'Image size mismatch:' + str(dirty.shape) + " != " + str(clean.shape)
        except Exception as e:
            rospy.logerr(str(e))
            return
        self.dirty = dirty
        self.updated = False

    def kMeans(self, K, Z):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1) #TODO: test different iteration counts...
        ret, label, center = cv2.kmeans(Z, K, criteria, 100, cv2.KMEANS_RANDOM_CENTERS)

        bboxes = []
        maxDim = 0
        for j in range(K):
            A = Z[label.ravel() == j]
            
            x_min=int(A[:,0].min())
            x_max=int(A[:,0].max())
            if (x_max - x_min > maxDim):
                maxDim = x_max - x_min

            y_min=int(A[:,1].min())
            y_max=int(A[:,1].max())
            if (y_max - y_min > maxDim):
                maxDim = y_max - y_min

            bboxes.append(((y_min,x_min), (y_max,x_max)))
        return (maxDim, bboxes)



    def update(self):
        rospy.loginfo('Updating')
        if self.updated:
            return
        #self.updated = True

        dirty = self.dirty
        clean = self.clean
        h,w,depth = self.shape

        threshold = 60
        Z = []

        #TODO: gaussian filtering to avoid single-pixel defects
        #TODO: mask over saturated values to avoid false hue values
        dirtyhsv = cv2.cvtColor(dirty,cv2.COLOR_BGR2HSV)
        cleanhsv = cv2.cvtColor(clean,cv2.COLOR_BGR2HSV)
        diffhsv = cv2.absdiff(dirtyhsv, cleanhsv)
        diffH = cv2.split(diffhsv)[0]

        for row in range(h):
            for col in range(w):
                if rospy.is_shutdown():
                    sys.exit(0)          
                if diffH[row,col] > threshold:
                    Z.append([row,col])
        if not len(Z):
            rospy.loginfo('Clean surface, no defects found')
            return

        #matrix cast for numpy
        Z = np.matrix(Z, dtype=np.float32)

        # how to determine k value by using k-means clustering method
        #TODO: perameterize max box size
        boxMax = 100
        rospy.loginfo('Running K-Means')

        #bboxes = self.kMeans(self.K, Z)
        maxDim, bboxes = self.kMeans(self.K, Z)

        #make sure K groupings are small enough
        while maxDim > boxMax and not rospy.is_shutdown():
            rospy.logdebug('Updating K value: ' + str(self.K))
            self.K += 1 
            maxDim, bboxes = self.kMeans(self.K, Z)

        #output markers
        rospy.loginfo('Dumping Markers')
        canvas = np.copy(diffhsv)
        for b in bboxes:
            # # test
            cv2.rectangle(canvas,b[0],b[1],(0,0,255),2)
            ######################################
            # for the markers part
            ######################################
            """marker = Marker()
            marker.header.frame_id = "world"  
            marker.header.stamp = rospy.Time()
            marker.ns = "flaws"
            marker.id = j
            marker.type = marker.CUBE
            marker.action = marker.ADD
            #TODO: pull these scale values from GUI params
            marker.pose.position.y = - float(x_max+x_min-h)/2/h
            marker.pose.position.x = 3*float(y_max+y_min-w)/2/w
            marker.pose.orientation.w = 1.0
            marker.scale.y = float(x_max-x_min)/h
            marker.scale.x = 3*float(y_max-y_min)/w
            marker.scale.z = 0.01
            marker.color.a = 0.75
            marker.color.r = 1.0
          
            # print marker
            self.defect_pub.publish(marker)"""
        #print('Done')
        cv2.imshow('Defects Found',canvas)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('flaw_detect')
    flaw_detect = FlawDetector()
    # worldImage_sub = rospy.Subscriber("dirty_map", Image, flaw_detect.callback)
    #TODO: perameterize rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        flaw_detect.update()
        rate.sleep()
    cv2.destroyAllWindows()



