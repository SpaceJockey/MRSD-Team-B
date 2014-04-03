#!/usr/bin/env python

### create this cvbroadcaster by following through 
### http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
### Songjie Zhong
### 04/01/2014
### MRSD project spacejocky

import roslib
roslib.load_manifest('roscv')
import sys
import rospy
import cv2
from std_msgs.msg import String
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
  def __init__(self):

    # Image data format from sensor_msgs.msg
        #     header: 
        #   seq: 0
        #   stamp: 
        #     secs: 0
        #     nsecs: 0
        #   frame_id: ''
        # height: 0
        # width: 0
        # encoding: ''
        # is_bigendian: 0
        # step: 0
        # data: []

    # img_msg = sensor_msgs.msg.Image()
    # img_msg.width = 640
    # img_msg.height = 480
    # img_msg.encoding = "rgba8"
    # img_msg.step = 640*4
    # img_msg.data = (640 * 480) * "1234"

    self.image_pub = rospy.Publisher("image_topic_2",Image)
    #cv.NamedWindow("Image window", 1)
    self.bridge_= CvBridge()
    self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

  def callback(self,data):
    try:
      cv2_image = self.bridge_.imgmsg_to_cv2(data, "rgb8")
      # information from that class
      # im = np.ndarray(shape=(img_msg.height, img_msg.width, n_channels),
      #                      dtype=dtype, buffer=img_msg.data)
      # if desired_encoding == "passthrough":
      #       return im

    except CvBridgeError, e:
      print e

      # assume here cv2_image is the OpenCV image formart i want for
      # the following operations for the image comparison

    # # img1 =cv2_image1
    # # img2 =cv2_image2
    # img1 = cv2.imread('testsurface_new1.png')  
    # img2 = cv2.imread('testsurface_baseline.png')
    # #print img1.shape       
    # gray1=cv2.cvtColor(img1,cv2.COLOR_RGB2GRAY)
    # print gray1[1,1]
    # gray2=cv2.cvtColor(img2,cv2.COLOR_RGB2GRAY)
    # print gray2[400,400]
    # #print gray1.shape

    # # Initiate SIFT detector
    # sift = cv2.SIFT()
    # # find the keypoints and descriptors with SIFT
    # kp1,des1=sift.detectAndCompute(gray1,None)
    # kp2,des2=sift.detectAndCompute(gray2,None)

    # # draw key points
    # #img1_1=cv2.drawKeypoints(gray1,kp1)
    # #img2_2=cv2.drawKeypoints(gray2,kp2)

    # #print img1.shape
    # #cv2.imwrite('img1_1.png',img1_1)
    # #cv2.imwrite('img2_2.png',img2_2)

    # MIN_MATCH_COUNT = 10

    # FLANN_INDEX_KDTREE = 0

    # index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    # search_params = dict(checks = 50)

    # flann = cv2.FlannBasedMatcher(index_params, search_params)

    # matches = flann.knnMatch(des1,des2,k=2)

    # # store all the good matches as per Lowe's ratio test.
    # good = []
    # for m,n in matches:
    #     if m.distance < 0.8*n.distance:
    #         good.append(m)

    # print len(good)

    # if len(good)>MIN_MATCH_COUNT:
    #     src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
    #     dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

    #     M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    #     print M

    #     #h=img2_2.shape[0]
    #     # print h 
    #     #w=img2_2.shape[1]
    #     h=gray2.shape[0]
    #     w=gray2.shape[1]
    #     print h,w
    #     # h=446, w=642

    # else:
    #     print "Not enough matches are found - %d/%d" % (len(good),MIN_MATCH_COUNT)
    #     matchesMask = None

    # dst = cv2.warpPerspective(img1,M,(w,h))
    # #dst = cv2.warpPerspective(gray1,M,(w,h))

    # # print pixel value rgb
    # # dst is the new defect image after warping
    # # print dst[1,1]
    # threshold=10
    # # a=dst[0,0]-gray2[0,0]
    # # print a
    # # b=dst[445,641]-gray2[445,641]
    # # print b
    # #b=dst[]
    # #print dst[1,1][0] # print value of r
    # for row in xrange(0,h):
    #   for col in xrange(0,w):
    #     if(((dst[row,col][0]-img2[row,col][0])*(dst[row,col][0]-img2[row,col][0])+(dst[row,col][1]-img2[row,col][1])*(dst[row,col][1]-img2[row,col][1])+(dst[row,col][2]-img2[row,col][2])*(dst[row,col][2]-img2[row,col][2]))>threshold):
    #       dst[row,col]= [255,0,0]     
    #     else:
    #       pass
    # # show images on the window
    # cv2.imshow('defect_image',dst)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # # print dst
    # plt.subplot(121),plt.imshow(img2),plt.title('Input')
    # plt.subplot(122),plt.imshow(dst),plt.title('Output')
    # plt.show()
    # # import IPython
    # # IPython.embed()


    #after operations doing in opencv
    # give the new value cv2_image1 and cv2_image2
    # back to convert to ROS images
    
    try:
      self.image_pub.publish(self.bridge_.cv2_to_imgmsg(cv2_image, "rgb8"))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




# Original code from the website!

# import roslib
# roslib.load_manifest('roscv')
# import sys
# import rospy
# import cv
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

# class image_converter:
#   def __init__(self):
#     self.image_pub = rospy.Publisher("image_topic_2",Image)
#     cv.NamedWindow("Image window", 1)
#     self.bridge = CvBridge()
#     self.image_sub = rospy.Subscriber("image_topic",Image,self.callback)

#   def callback(self,data):
#     try:
#       cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
#     except CvBridgeError, e:
#      print e

#     (cols,rows) = cv.GetSize(cv_image)
#     if cols > 60 and rows > 60 :
#       cv.Circle(cv_image, (50,50), 10, 255)
#     cv.ShowImage("Image window", cv_image)
#     cv.WaitKey(3)

#     try:
#       self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
#     except CvBridgeError, e:
#       print e

# def main(args):
#   ic = image_converter()
#   rospy.init_node('image_converter', anonymous=True)
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print "Shutting down"
#   cv.DestroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)