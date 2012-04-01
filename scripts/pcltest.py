#!/usr/bin/env python
import sys

import roslib; roslib.load_manifest('comp558')
import rospy, rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv
from sensor_msgs.msg import Image, PointCloud2
from comp558 import point_cloud


filename = '/media/058e56e8-baba-338d-805f-16c587700d1b/stuff/2012-03-16-00-00-12.bag' 
if len(sys.argv) > 1: filename = sys.argv[1]

depth_topic = "/camera/depth/image"
rgb_topic = "/camera/rgb/image_color"
pcl_topic = "/camera/depth/points"


try:
   bag = rosbag.Bag(filename)

   print "Reading..."
#   depth_count = 0
#   rgb_count = 0
#   pcl_count = 0
#   for topic, msg, t in bag.read_messages(topics=[depth_topic, rgb_topic, pcl_topic]):
#      # count the messages
#      if topic == depth_topic: 
#	 depth_count += 1
#	 print "depth: %s" % msg.header.stamp.to_nsec()
#      elif topic == rgb_topic: rgb_count += 1
#      elif topic == pcl_topic: 
#	 pcl_count += 1
#	 print "pcl: %s" % msg.header.stamp.to_nsec()
#	 # test the point_cloud.read_points thing
#	 for pt in point_cloud.read_points(msg):
#	    print pt
#	    raw_input()
#   print "rgb: %s depth: %s pcl: %s" % (rgb_count, depth_count, pcl_count)

   # get a pcl and a depth message with the same timestamp and compare point
   d_msg = None
   p_msg = None
   for topic, msg, t in bag.read_messages(topics=[depth_topic, rgb_topic, pcl_topic]):
      if topic == depth_topic: d_msg = msg
      elif topic == pcl_topic: p_msg = msg
      if d_msg != None and p_msg != None and d_msg.header.stamp.to_nsec() == p_msg.header.stamp.to_nsec():
	 break

   # convert Image to cv format
   bridge = CvBridge()
   im = bridge.imgmsg_to_cv(d_msg)

   # print out depth values and hope for the best
   count = 0
   ptmax = 2000
   for v in point_cloud.read_points(p_msg):
      print v
      count += 1
      if count > ptmax: break
   count = 0
   for i in xrange(im.rows):
      for j in xrange(im.cols):
	 count += 1
	 if count > ptmax: break
	 print im[i,j]
      if count > ptmax: break


finally:
   bag.close()
