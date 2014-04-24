#!/usr/bin/env python
####   Inspection for spacejockey
####   Songjie Zhong
####   03/19/2014
####   update date 04/13/2014
####   add k-means for the bonding box and center informations
####   this is the version working for whole pipeline
####   dirty is dirty map from rostopic, clean is clean map from reading png in the folder

import numpy as np
import cv2
from matplotlib import pyplot as plt
import os,sys
import urllib 
from sensor_msgs.msg import Image
import roslib
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError 
import geometry_msgs.msg
import copy

# for the maker part
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class ImageComparison(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.defect_pub = rospy.Publisher('/visualization_marker', Marker)
        # call the dirty_map
        #self.worldImage_sub = rospy.Subscriber("dirty_map",Image,self.callback)

        #test data
        self.dirty = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_baseline.png") 
        self.clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../test/testsurface_baseline.png") 

        #real data
        #TODO: perameterize config file name
        #self.clean = cv2.imread(os.path.dirname(sys.argv[0])+"/../config/clean_map.png")
        self.height, self.width, self.depth = self.clean.shape

    def callback(self,data):
        clean = self.clean
        dirty = self.dirty
        """
        try:
            dirty = self.bridge.imgmsg_to_cv2(data, "passthrough")
            if(dirty.shape != clean.shape):
                raise Exception('Image size mismatch:' + str(dirty.shape) + " != " + str(clean.shape))
        except Exception as e:
            rospy.logerr(str(e))
            return
        """
        # set up the threshold
        threshold=10
        Z=[]
        
        ## void the edge's influnce, so +-10 for the row, col number
        for row in xrange(10+0,self.height-10):
            for col in xrange(10+0,self.width-10):
                if(((dirty[row,col][0]-clean[row,col][0])*(dirty[row,col][0]-clean[row,col][0])+(dirty[row,col][1]-clean[row,col][1])*(dirty[row,col][1]-clean[row,col][1])+(dirty[row,col][2]-clean[row,col][2])*(dirty[row,col][2]-clean[row,col][2]))>threshold):
                    dirty[row,col]= [255,0,0]
                    Z = Z+[row,col]
                else:
                    pass

        # N is the number of the points
        N=len(Z)/2
        Z=np.matrix(Z)
        Z=Z.reshape((N,2))
        Z=np.float32(Z)

        #Tune K value k-means clustering method
        #TODO: perameterize boxMaxLength!
        K = 6
        """
        # make the decision whether count for one group or not compare the maxlength of the bonding box
        boxMaxLength=60
        while(1):
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
                K = K+1
                continue
            else:
                break
                """

        # then do the k-means clustering again with most suitable K value from above
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        ret, label, center = cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)
        #print "center information of all the groups:......"
        #print center
        for j in xrange(K):
            A = Z[label.ravel() == j]
            #print A[:,1]
            y_max=int(A[:,1].max())
            y_min=int(A[:,1].min())
            x_max=int(A[:,0].max())
            x_min=int(A[:,0].min())
            # draw rectangle's function
            #cv2.rectangle(dst,(y_min,x_min),(y_max,x_max),(0,255,0),3)  

            ######################################
            # for the markers part
            ######################################
            marker = Marker()
            marker.header.frame_id = "world"  
            marker.header.stamp = rospy.Time.now()
            marker.ns = "flaws"
            marker.id = j
            marker.type = marker.CUBE
            #TODO: pull these from GUI perameters
            marker.pose.position.y = float(x_max+x_min-self.height)/2/self.height
            marker.pose.position.x = 3 * float(y_max+y_min-self.width)/2/self.width
            marker.pose.orientation.w = 1.0
            marker.scale.y = float(x_max-x_min)/self.width
            marker.scale.x = 3*float(y_max-y_min)/self.height
            marker.scale.z = 0.01
            marker.color.a = 0.75
            marker.color.r = 1.0
            self.defect_pub.publish(marker)

        # print dirty
        #plt.subplot(121),plt.imshow(clean),plt.title('Input')
        #plt.subplot(122),plt.imshow(dirty),plt.title('Output')
        #plt.show()

if __name__ == '__main__':
  rospy.init_node('flaw_detector', anonymous=True)
  flaw_detector = ImageComparison()
  rospy.sleep(3)
  flaw_detector.callback(None)
  rospy.spin() 