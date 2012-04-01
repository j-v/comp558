#!/usr/bin/env python
import roslib
roslib.load_manifest('comp558')
import sys
import rospy
import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # used for bridge that converts ROS Image msg to OpenCV IpIImage

class ImageViewer:

   def __init__(self, topic):
      self.topic = topic
      self.subscriber = rospy.Subscriber(topic, Image, self.callback)
      cv.NamedWindow("Image window", 1)
      self.bridge = CvBridge()

   def callback(self,data):
      try:
        cv_image = self.bridge.imgmsg_to_cv(data) 
      except CvBridgeError, e:
        print "ERROR = " + e

      # rescale intensity of image to be between 0 and 1
      n_img = cv.CreateMat(cv_image.rows, cv_image.cols, cv_image.type)
      cv.ConvertScale(cv_image,n_img,0.1)
      cv.ShowImage("Image window", n_img)
      cv.WaitKey(3)

def main(args):
   image_topic = args[1]
   iv = ImageViewer(image_topic)
   rospy.init_node('image_viewer', anonymous=True)
   rospy.spin()
   print "Shutting down"

   cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
