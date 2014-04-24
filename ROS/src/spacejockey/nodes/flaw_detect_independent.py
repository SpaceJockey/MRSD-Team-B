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
        self.worldImage_sub = rospy.Subscriber("camera/image_raw",Image,self.callback)
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except Exception as e:
            rospy.logerr(str(e))
            return

        img1 = cv2.imread(os.path.dirname(sys.argv[0]) +'/testsurface_new1.png')  
        h,w,depth=img1.shape
        img2 = cv2.imread(os.path.dirname(sys.argv[0]) +'/testsurface_baseline.png')

        threshold=0
        Z=[]
        ## void the edge's influnce, so +-10 for the row, col number
        for row in xrange(10+0,h-10):
            for col in xrange(10+0,w-10):
                ### hard code the size of the april tags here 
                ## in order to void the influence from april tags
                if ((row>=0 and row<=120) and (col>=0 and col<=120)) or ((row>=0 and row<=120) and (col>=(w-120) and col<=w)) or ((row>=(h-120) and row<=h) and (col>=0 and col<=120)) or ((row>=(h-120) and row<=h) and (col>=(w-120) and col<=w)):
                    pass
                else:
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

        # how to determine k value by using k-means clustering method
        # start trying with k=1
        K = 1
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
            if count >0:
                K=K+1
                continue
            else:
                break


        # then do the k-means clustering again with most suitable K value from above
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.1)
        ret,label,center=cv2.kmeans(Z,K,criteria,100,cv2.KMEANS_RANDOM_CENTERS)
        # print "center information of all the groups:......"
        # print center
        for j in xrange(K):
            A = Z[label.ravel()==j]
            #print A[:,1]
            y_max=int(A[:,1].max())
            y_min=int(A[:,1].min())
            x_max=int(A[:,0].max())
            x_min=int(A[:,0].min())
            # draw rectangle's function
            cv2.rectangle(img1,(y_min,x_min),(y_max,x_max),(0,255,0),3)

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
            marker.pose.position.y = float(x_max+x_min-h)/2/h
            marker.pose.position.x = 3*float(y_max+y_min-w)/2/w
            marker.pose.orientation.w = 1.0
            marker.scale.y = float(x_max-x_min)/h
            marker.scale.x = 3*float(y_max-y_min)/w
            marker.scale.z = 0.01
            marker.color.a = 0.75
            marker.color.r = 1.0
          
            # print marker
            self.defect_pub.publish(marker)
     
          
        # # show images on the window
        # cv2.imshow('defect_image',img1)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


if __name__ == '__main__':
  rospy.init_node('ImageComparison_listener', anonymous=True)
  ImageComparison_listener = ImageComparison()
  rospy.spin() 
  cv2.destroyAllWindows()



