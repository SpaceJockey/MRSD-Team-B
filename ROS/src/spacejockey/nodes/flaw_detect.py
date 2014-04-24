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

class ImageComparison(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.defect_pub = rospy.Publisher('/visualization_marker', Marker)

        #test data
        #self.dirty = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_dirty.png") 
        #self.clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_clean.png") 

        #real data
        #TODO: perameterize config file name
        self.clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../config/clean_map.png")
        self.dirty = np.zeros(self.clean.shape, dtype=np.uint8)
        self.shape = self.clean.shape
        self.updated = False


    def callback(self,data):
        clean = self.clean
        try:
            dirty = self.bridge.imgmsg_to_cv2(data, "passthrough")
            if(dirty.shape != clean.shape):
                raise Exception('Image size mismatch:' + str(dirty.shape) + " != " + str(clean.shape))
        except Exception as e:
            rospy.logerr(str(e))
            return
        self.dirty = dirty
        self.updated = False

    def update(self):
        if self.updated:
            return
        self.updated = True
        dirty = self.dirty
        clean = self.clean
        h,w,depth = self.shape

        #todo: perameterize threshold
        threshold=100
        Z=[]

        dirtyGray = cv2.cvtColor(dirty,cv2.COLOR_BGR2GRAY)
        cleanGray = cv2.cvtColor(clean, cv2.COLOR_BGR2GRAY)
        ret, dirtyMask = cv2.threshold(dirtyGray, 10, 255, cv2.THRESH_BINARY)

        r1 = cv2.subtract(dirty, clean, mask=dirtyMask)  
        r2 = cv2.subtract(clean, dirty, mask=dirtyMask)
        #absD = cv2.absdiff(dirty, clean)
        ret, diff = cv2.threshold(cv2.add(r1, r2), threshold, 255, cv2.THRESH_BINARY) 
        
        #FIXME: this is still freakin slow!
        for row in range(h):
            for col in range(w):
                if rospy.is_shutdown():
                    sys.exit(0)             
                if diff[row,col,0]: 
                    Z.append([row,col])
        if not len(Z):
            rospy.loginfo('Clean surface, no defects found')
            return

        #matrix cast for numpy
        Z = np.matrix(Z, dtype=np.float32)

        # how to determine k value by using k-means clustering method
        #TODO: perameterize max box size
        K = 1
        boxMaxLength=60
        max_K = 5
        rospy.loginfo('Calculating new K')
        while not rospy.is_shutdown():
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
            ret,label,center=cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)
            count=0
            for j in xrange(K):
                A = Z[label.ravel()==j]
                #print A[:,1]
                y_max=int(A[:,1].max())
                y_min=int(A[:,1].min())
                x_max=int(A[:,0].max())
                x_min=int(A[:,0].min())
                # print y_max,y_min,x_max,x_min
                # draw rectangle's function
                if (y_max-y_min)>boxMaxLength or (x_max-x_min)>boxMaxLength:
                    count=count+1
            if count > 0:
                K=K+1
                if K > max_K:
                    break
            else:
                break


        # then do the k-means clustering again with most suitable K value from above
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        ret,label,center=cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)
        rospy.loginfo('Dumping Markers')
        for j in xrange(K):
            A = Z[label.ravel()==j]
            #print A[:,1]
            y_max=int(A[:,1].max())
            y_min=int(A[:,1].min())
            x_max=int(A[:,0].max())
            x_min=int(A[:,0].min())

            ######################################
            # for the markers part
            ######################################
            marker = Marker()
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
            self.defect_pub.publish(marker)
        print('Done')

if __name__ == '__main__':
    rospy.init_node('flaw_detect')
    flaw_detect = ImageComparison()
    worldImage_sub = rospy.Subscriber("dirty_map", Image, flaw_detect.callback)
    #TODO: perameterize rate
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        flaw_detect.update()
        rate.sleep()




