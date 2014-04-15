#!/usr/bin/env python
####   Inspection for spacejockey
####   Songjie Zhong
####   03/19/2014
####   update date 04/13/2014
####   add k-means for the bonding box and center informations

#### compare the difference of the two world images

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
        # call the worldImage 
        self.worldImage_sub = rospy.Subscriber("spacecam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except Exception as e:
            rospy.logerr(str(e))
            return
        
        # TODO
        # need to call the worldImage for image comparison
        # right now whole pipeline is not finished
        # so test here by using images created before for image comparison testing

        # here should have img1,img2
        img1=cv_image
        img2=cv_image
        # how to control different time to catch infor from ROSimages msgs
        # to get different time status worldImages
        height, width, depth = img1.shape
        print height

        # print "Inspection is working..."
        # print 'sys.argv[0] =', sys.argv[0]
        # print 'path = ' + os.path.dirname(sys.argv[0]) 

        # img1 = cv2.imread('testsurface_new1.png')  
        # print type(img1)
        # img2 = cv2.imread('testsurface_baseline.png')

        # print img1.shape
        gray1=cv2.cvtColor(img1,cv2.COLOR_RGB2GRAY)
        gray2=cv2.cvtColor(img2,cv2.COLOR_RGB2GRAY)
        #print gray2[400,400]
        #print gray1.shape

        # Initiate SIFT detector
        sift = cv2.SIFT()
        # find the keypoints and descriptors with SIFT
        kp1,des1=sift.detectAndCompute(gray1,None)
        kp2,des2=sift.detectAndCompute(gray2,None)

        # draw key points
        #img1_1=cv2.drawKeypoints(gray1,kp1)
        #img2_2=cv2.drawKeypoints(gray2,kp2)

        #print img1.shape
        #cv2.imwrite('img1_1.png',img1_1)
        #cv2.imwrite('img2_2.png',img2_2)

        MIN_MATCH_COUNT = 10

        FLANN_INDEX_KDTREE = 0

        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv2.FlannBasedMatcher(index_params, search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # store all the good matches as per Lowe's ratio test.
        good = []
        for m,n in matches:
            if m.distance < 0.8*n.distance:
                good.append(m)

        #print len(good)

        if len(good)>MIN_MATCH_COUNT:
            src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
            dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
            print M

            #h=img2_2.shape[0]
            # print h 
            #w=img2_2.shape[1]
            h=gray2.shape[0]
            w=gray2.shape[1]
            print h,w
            # h=600, w=600
            # value from the size of the world map

        else:
            print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
            matchesMask = None

        dst = cv2.warpPerspective(img1,M,(w,h))
        #dst = cv2.warpPerspective(gray1,M,(w,h))

        # print pixel value rgb
        # dst is the new defect image after warping
        # print dst[1,1]
        threshold=10
        # a=dst[0,0]-gray2[0,0]
        # print a
        # b=dst[445,641]-gray2[445,641]
        # print b
        #b=dst[]
        #print dst[1,1][0] # print value of r


        Z=[]
        ## void the edge's influnce, so +-10 for the row, col number
        for row in xrange(10+0,h-10):
            for col in xrange(10+0,w-10):
                ### hard code the size of the april tags here 
                ## in order to void the influence from april tags
                if ((row>=0 and row<=120) and (col>=0 and col<=120)) or ((row>=0 and row<=120) and (col>=(w-120) and col<=w)) or ((row>=(h-120) and row<=h) and (col>=0 and col<=120)) or ((row>=(h-120) and row<=h) and (col>=(w-120) and col<=w)):
                    pass
                else:
                    if(((dst[row,col][0]-img2[row,col][0])*(dst[row,col][0]-img2[row,col][0])+(dst[row,col][1]-img2[row,col][1])*(dst[row,col][1]-img2[row,col][1])+(dst[row,col][2]-img2[row,col][2])*(dst[row,col][2]-img2[row,col][2]))>threshold):
                        dst[row,col]= [255,0,0]
                        Z=Z+[row,col]
                    else:
                        pass

        ## N is the number of the points
        N=len(Z)/2
        #print N
        Z=np.matrix(Z)
        Z=Z.reshape((N,2))
        #print Z
        Z=np.float32(Z)

        ## we can first see the result of the image comparion 
        ## to ge the K=?

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        K = 8
        ret,label,center=cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)

        for j in xrange(K):
            A = Z[label.ravel()==j]
            #print A[:,1]
            y_max=int(A[:,1].max())
            y_min=int(A[:,1].min())
            x_max=int(A[:,0].max())
            x_min=int(A[:,0].min())
            #print y_max,y_min,x_max,x_min
            #### draw rectangle's function
            #print dst[110,1]
            cv2.rectangle(dst,(y_min,x_min),(y_max,x_max),(0,255,0),3)

            #print dst[110,1]

        print "center information of all the groups:......"
        print center
        # # Plot the data
        # print center[:,1]
        # print center[:,0]

        # show images on the window
        cv2.imshow('defect_image',dst)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # print dst
        plt.subplot(121),plt.imshow(img2),plt.title('Input')
        plt.subplot(122),plt.imshow(dst),plt.title('Output')
        plt.show()

if __name__ == '__main__':
  rospy.init_node('ImageComparison_listener', anonymous=True)
  ImageComparison_listener = ImageComparison()
  rospy.spin() 
  cv2.destroyAllWindows()


# import IPython
# IPython.embed()


