#!/usr/bin/env python
import sys

import roslib; roslib.load_manifest('comp558')
import rospy, rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv


# synctest.py
# jvolkmar 2012
# Test to see how in sync the depth/rgb images are from the ros bag taken with the kinect
# Outputs the framerate of received RGB and Depth images from the Kinect sensor.



def receive_data(filename) :
   depth_topic = "/camera/depth/image"
   rgb_topic = "/camera/rgb/image_color"

   print "Reading data from %s" % filename
   print "RGB Image on topic: " + rgb_topic
   print "Depth Image on topic: " + depth_topic

   lastImgTime = -1 

   rgb_start = 0
   depth_start = 0
   rgb_count = 0
   depth_count = 0

   largestDiff = float("-inf")
   smallestDiff = float("inf")
   diff_accum = 0

   minDepth = float("inf")
   maxDepth = float("-inf")

   bridge = CvBridge()

   try:
      bag = rosbag.Bag(filename)

      print "Reading..."
      for topic, msg, t in bag.read_messages(topics=[depth_topic, rgb_topic]):
	 # print "%s %s" % (topic, t)
	 if topic == rgb_topic: # read RBG image
	    if (rgb_start==0): rgb_start = t.to_nsec()
	    lastImgTime = t.to_nsec()
	    rgb_count += 1
	 else: # read Depth image
	    if depth_start ==0: depth_start = t.to_nsec()
	    lastDepthTime = t.to_nsec()
	    depth_count += 1
	    depth_image = bridge.imgmsg_to_cv(msg)
	    (minVal, maxVal, _, _) = cv.MinMaxLoc(depth_image)
	    if minVal < minDepth: minDepth = minVal
	    if maxVal > maxDepth: maxDepth = maxVal
	    if lastImgTime != -1:
	       diff = t.to_nsec() - lastImgTime
	       # Optional: print temporal alignment difference of depth and image frames in nanoseconds
	       # at every depth frame
	       #print "diff: %s" % (diff,) 
	       if diff < smallestDiff: smallestDiff = diff
	       if diff > largestDiff: largestDiff = diff
	       diff_accum += diff

      a_billion = 1000000000.0

      rgb_elapsed = (lastImgTime - rgb_start)/ a_billion # seconds
      depth_elapsed = (lastDepthTime - depth_start) / a_billion # seconds
      rgb_rate = float(rgb_count) / float(rgb_elapsed)
      depth_rate = float(depth_count) / float(depth_elapsed)

      average_diff = (float(diff_accum) / float(depth_count)) / a_billion
      # TODO average_diff could be improved by taking image pair of min_diff( (rgb,depth), (depth,rgb) )
      # instead of always taking diff(rgb,depth)

      elapsed = (max(lastImgTime,lastDepthTime) - min(rgb_start, depth_start)) / a_billion # seconds

      print "Duration of " + filename + ": " + str(elapsed) + " seconds"
      print "RGB %s + Depth %s = TOTAL %s frames read" % (rgb_count, depth_count, rgb_count+depth_count)
      print "Recorded RGB framerate: %s FPS \nRecorded depth framerate: %s FPS" % (rgb_rate, depth_rate)
      print "Average sync difference %s sec" % average_diff
      print "maximum diff: %s sec \nminimum diff: %s sec" % (float(largestDiff)/a_billion, 
	    						     float(smallestDiff)/a_billion)
      print "Min Depth: %s Max Depth: %s" % (minDepth, maxDepth)
   except Exception, e:
      print "ERROR: " + str(e)
   finally:
      bag.close()

def main(args):

   filename = '/media/058e56e8-baba-338d-805f-16c587700d1b/stuff/2012-03-16-00-00-12.bag' 

   if len(args) > 1: filename = args[1]

   def print_usage():
      print args[0] + """ [bag_filename]
Test to see how in sync the depth/rgb images are from the ros bag taken with the kinect.
Outputs the framerate of received RGB and Depth images from the Kinect sensor.
bag_filename: path to the bag file to analyze
""" 
   receive_data(filename)

   exit()



if __name__ == '__main__':
    main(sys.argv)
