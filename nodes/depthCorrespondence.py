#!/usr/bin/env python
import roslib; roslib.load_manifest('comp558')
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
import cv
import math
import sys

#import logging as log
#log.basicConfig(filename="depth_correspondence.log",level=log.DEBUG)

depth_topic = "/camera/depth/image"
rgb_topic = "/camera/rgb/image_color"


# Given depth images Dold and Dnew, optical flow
# components X and Y, and temporal change deltaT,
# give the Z flow estimate
# images expected in CvMat format
def getDepthChanges(Dold, Dnew, X, Y):
   rospy.loginfo('calculating depth change')
   Z = cv.CreateMat(X.rows, X.cols, Dold.type)
   for i in range(X.rows):
      for j in range(X.cols):
	 iDiff = round(X[i,j])
	 jDiff = round(Y[i,j])
	 iOld = i - iDiff
	 jOld = j - jDiff
	 # handle out of bounds TODO think of a better way
	 if iOld < 0: iOld = 0
	 if iOld >= X.rows: iOld = X.rows-1
	 if jOld < 0: jOld = 0
	 if jOld >= X.cols: jOld = X.cols-1

	 Z[i,j] = Dnew[i,j] - Dold[iOld, jOld]
	 
	 # TODO  avoid nan values in Z: currently some values will be
	 # nan if the corresponding values in Z or Zold are nan
   
   return Z

def avg_masking_nan(M):
   mask = cv.CreateMat(M.rows, M.cols, cv.CV_8U)
   for i in range(M.rows):
      for j in range(M.cols):
	 if not math.isnan(M[i,j]): mask[i,j] = 1
	 else: mask[i,j] = 0

   mean = cv.Avg(M, mask)[0] # index at Zero, b/c cv.Avg returns a 4-tuple for some reason
   return mean

class DepthChangeEstimator:

   def __init__(self, node_name):
      rospy.loginfo('Starting node ' + node_name)
      rospy.init_node(node_name, log_level=rospy.DEBUG)
      rospy.loginfo('node started')

      self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback)
      self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback)

      self.marker_pub = rospy.Publisher('/depth_change_marker_array', MarkerArray)

      self.lastRGBImage = None
      self.lastDepthImage = None
      self.velX = cv.CreateMat(480,640, cv.CV_32FC1) # the type is required by CalcOpticalFlowLK as a destination matrix
      self.velY = cv.CreateMat(480,640, cv.CV_32FC1)

   def rgb_callback(self, img_message):
      rospy.loginfo('Received rgb image')
      #import pdb; pdb.set_trace()
      # convert to openCV format
      bridge = CvBridge()
      try:
	 timestamp = img_message.header.stamp # time in rospy.Time format
	 rgb_image = bridge.imgmsg_to_cv(img_message, "mono8")
	 if self.lastRGBImage != None:
	    # do the optical flow estimation
	    # velocity components of each pixel are saved to self.velX and self.velY
	    window_size = (5,5) # must be odd
	    cv.CalcOpticalFlowLK(self.lastRGBImage, rgb_image, window_size, self.velX, self.velY)
	 self.lastRGBImage = rgb_image
      except CvBridgeError, e:
	 rospy.logerr(e)

   def depth_callback(self, img_message):
      rospy.loginfo('Received depth image')
      bridge = CvBridge()
      try:
         timestamp = img_message.header.stamp
	 depth_img = bridge.imgmsg_to_cv(img_message)
	 if self.lastDepthImage != None and self.velX != None and self.velY != None:
	    # do the depth change estimate
	    dZ = getDepthChanges(self.lastDepthImage, depth_img, self.velX, self.velY)
	    #import pdb; pdb.set_trace()
	    # TODO publish rviz arrow visualization messages

	 self.lastDepthImage = depth_img
      except CvBridgeError, e:
	 rospy.logerr(e)
	
   def publish_markers(self, dX, dY, Zcurr, Zprev, region_size):
      """ Publish arrow markers to rviz showing pixel-wise velocity estimate, 
      averaging over neighborhoods in the depth/rgb image pairs
      Args:
      dX: pixel-wise velocity estimate on X axis
      dY: pixel-wise velocity estimate on Y axis
      Zcurr: depth image for current frame 
      Zprev: depth image for previous frame
      region_size: pixel-size of square neigborhood"""
      pass
      # TODO code this up. need to find correspondence between 2d pixels and 3d coords...







def main(args):
   node_name = "depth_change_estimator"
   DepthChangeEstimator(node_name)

   rospy.spin()
   rospy.loginfo('node ' + node_name + ' terminated')

if __name__ == '__main__':
    main(sys.argv)
