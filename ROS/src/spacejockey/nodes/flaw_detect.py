#!/usr/bin/env python
####   Inspection for spacejockey
####   Songjie Zhong
####   03/19/2014
####   update date 04/13/2014
####   add k-means for the bonding box and center informations
####   this is the version working for whole pipeline
####   img1 is dirty map from rostopic, img2 is clean map from reading png in the folder

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

class ImageComparison(object):
    def __init__(self):
        self.bridge = CvBridge()
        # call the dirty_map
        self.worldImage_sub = rospy.Subscriber("dirty_map",Image,self.callback)
       
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except Exception as e:
            rospy.logerr(str(e))
            return
        # choice 1, conneted with whole pipepline
        # img1 is dirty map called from subscriber
        # img2 is baseline map stored befoe under the folder
        img1 = cv_image 
        img2 = cv2.imread(os.path.dirname(sys.argv[0])+"/World_Image.png") 
        h1, w1, depth1 = img1.shape
        h2, w2, depth2 = img2.shape
        # print h1,w1,depth1
        # print h2,w2,depth2
        # print img1
        # print img2

        # # check whether img1, img2 are passing to here
        # for row in range(h1):
        #     for col in range(w1):
        #         if img1[row,col][0]!=0 or img1[row,col][1]!=0 or img1[row,col][1]!=0:
        #             print img1[row,col]
        # # good ! there are values for img1
        # for row in range(h2):
        #     for col in range(w2):
        #         if img2[row,col][0]!=0 or img2[row,col][1]!=0 or img2[row,col][1]!=0:
        #             print img2[row,col]
        # good ! there are values for img2




        # we do not need to warp the img1 because img1 and img2 are already under same situation
        # following code directly begins with pixel value comparions

        # set up the threshold
        threshold=10
        Z=[]
        ## void the edge's influnce, so +-10 for the row, col number
        for row in xrange(10+0,h1-10):
            for col in xrange(10+0,w1-10):
                if(((img1[row,col][0]-img2[row,col][0])*(img1[row,col][0]-img2[row,col][0])+(img1[row,col][1]-img2[row,col][1])*(img1[row,col][1]-img2[row,col][1])+(img1[row,col][2]-img2[row,col][2])*(img1[row,col][2]-img2[row,col][2]))>threshold):
                    img1[row,col]= [255,0,0]
                    Z=Z+[row,col]
                else:
                    pass

        # N is the number of the points
        N=len(Z)/2
        Z=np.matrix(Z)
        Z=Z.reshape((N,2))
        Z=np.float32(Z)
        print Z

        # how to determine k value by using k-means clustering method
        # start trying with k=2
        K = 1
        # make the decision whether count for one group or not compare the maxlength of the bonding box
        boxMaxLength=60
        while(1):
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
            ret,label,center=cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)

            # print "center information of all the groups:......"
            # print center
            # print center[:,1]
            # print center[:,0]
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
            if count >0:
                K=K+1
                continue
            else:
                break
        print K

        # then do the k-means clustering again with most suitable K value from above
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        ret,label,center=cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)
        print "center information of all the groups:......"
        print center
        for j in xrange(K):
            A = Z[label.ravel()==j]
            #print A[:,1]
            y_max=int(A[:,1].max())
            y_min=int(A[:,1].min())
            x_max=int(A[:,0].max())
            x_min=int(A[:,0].min())
            # draw rectangle's function
            cv2.rectangle(dst,(y_min,x_min),(y_max,x_max),(0,255,0),3)  

        # show images on the window
        cv2.imshow('defect_image',img1)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # print img1
        plt.subplot(121),plt.imshow(img2),plt.title('Input')
        plt.subplot(122),plt.imshow(img1),plt.title('Output')
        plt.show()

if __name__ == '__main__':
  rospy.init_node('ImageComparison_listener', anonymous=True)
  ImageComparison_listener = ImageComparison()
  rospy.spin() 
  cv2.destroyAllWindows()


