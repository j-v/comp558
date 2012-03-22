#!/usr/bin/env python
import roslib
roslib.load_manifest('comp558')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # used for bridge that converts ROS Image msg to OpenCV IpIImage
import time

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv.NamedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_color",Image,self.callback)
    self.image_list = []
    self.prev = 0
    self.curr = 1
    self.velx = cv.CreateMat(480,640, cv.CV_32FC1) # the type is required by CalcOpticalFlowLK as a destination matrix
    self.vely = cv.CreateMat(480,640, cv.CV_32FC1)
    self.FOUT = open('./output.txt','w')
    '''
    import pdb; pdb.set_trace();
    self.cameraSize = cv.CvSize(640,480)
    self.cameraSize.width = 640
    self.cameraSize.height = 480
    '''

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv(data, "mono8") # This is where the conversion occurs. mono8 encoding is required by CalcOpticalFlowLK algo
    except CvBridgeError, e:
      print "ERROR = " + e

    rows = 480;
    cols = 640;
    (height,width) = cameraSize = cv.GetSize(cv_image)
    self.image_list.append(cv_image)


    if len(self.image_list) > 1:
       # do the KL here...
       cv.CalcOpticalFlowLK(self.image_list[self.prev], self.image_list[self.curr], (5,5), self.velx, self.vely) # not sure what window size is. but They must be odd-valued (5,5) was chosen randomly

       self.prev = self.prev+1
       self.curr = self.curr+1

    # show the velocity in every pixel
    # show the image in the image window by publishing it
    # cv.ShowImage("Image window", self.velx)
    # self.FOUT.write(str(time.clock()))
    #for i in range((self.velx).height):
    #   for j in range((self.vely).width):
    #		self.FOUT.write(str(self.velx[i,j]))

    cv.ShowImage("Image window", cv_image)
    cv.ShowImage("velx",self.velx)
    cv.ShowImage("vely",self.vely)

    # test: get min/max velx/vely vals
    minx = miny = float('inf')
    maxx = maxy = float('-inf')
    for i in xrange(self.velx.rows):
       for j in xrange(self.velx.cols):
	  x = self.velx[i,j]
	  y = self.vely[i,j]
	  if x < minx: minx=x
	  if x> maxx: maxx=x
	  if y < miny: miny=y
	  if y > maxy: maxy=y

    avgx = cv.Avg(self.velx)
    avgy = cv.Avg(self.vely)
    print "minx: %s maxx: %s miny: %s maxy: %s" % (minx, maxx, miny, maxy)
    print "avgx: %s avgy: %s" % (avgx, avgy)


    cv.WaitKey(3)
    #try:
    #  self.image_pub.publish(self.bridge.cv_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError, e:
    #  print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  time.clock();
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

  (ic.FOUT).close();
  import pdb; pdb.set_trace();
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
