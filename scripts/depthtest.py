#!/usr/bin/env python
import roslib; roslib.load_manifest('comp558')
import rospy, rosbag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv
import math

def avg_masking_nan(M):
   mask = cv.CreateMat(M.rows, M.cols, cv.CV_8U)
   for i in range(M.rows):
      for j in range(M.cols):
	 if not math.isnan(M[i,j]): mask[i,j] = 1
	 else: mask[i,j] = 0

   mean = cv.Avg(M, mask)[0] # index at Zero, b/c cv.Avg returns a 4-tuple for some reason

filename = '/media/JANGLES/2012-03-16-00-00-12.bag'

try:
   bag = rosbag.Bag(filename)
   bridge = CvBridge()

   for topic, msg, t in bag.read_messages(topics=['/camera/depth/image']):
      data = msg.data
      step = msg.step
      
      # calculate average depth
      print img

      mean = avg_masking_nan(img)
      print mean
      raw_input()

except Exception as e:
   print e
finally:
   bag.close()
