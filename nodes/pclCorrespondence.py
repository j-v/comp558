#!/usr/bin/env python
import roslib; roslib.load_manifest('comp558')
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import cv
import math
import sys
from comp558 import point_cloud
from itertools import izip


pcl_topic = "/camera/depth/points"
rgb_topic = "/camera/rgb/image_color"



def avg_masking_nan(M):
   mask = cv.CreateMat(M.rows, M.cols, cv.CV_8U)
   for i in xrange(M.rows):
      for j in xrange(M.cols):
	 if not math.isnan(M[i,j]): mask[i,j] = 1
	 else: mask[i,j] = 0

   mean = cv.Avg(M, mask)[0] # index at Zero, b/c cv.Avg returns a 4-tuple for some reason
   return mean

class DepthChangeEstimator:

   def __init__(self, node_name):
      rospy.loginfo('Starting node ' + node_name)
      rospy.init_node(node_name, log_level=rospy.DEBUG)
      rospy.loginfo('node started')

      self.pcl_sub = rospy.Subscriber(pcl_topic, PointCloud2, self.pcl_callback)
      self.rgb_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback)

      self.marker_pub = rospy.Publisher('/depth_change_marker_array', MarkerArray)

      self.lastRGBImage = None
      self.lastPointCloud = None
      self.velX = cv.CreateMat(480,640, cv.CV_32FC1) # the type is required by CalcOpticalFlowLK as a destination matrix
      self.velY = cv.CreateMat(480,640, cv.CV_32FC1)

      cv.NamedWindow('flow')

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
	    self.show_flow(rgb_image)
	 self.lastRGBImage = rgb_image
      except CvBridgeError, e:
	 rospy.logerr(e)



   def show_flow(self, mono_image):
      region_size = 64
      (rows, cols) = (self.velX.rows, self.velY.cols)
      flow_image = cv.CreateMat(rows, cols, cv.CV_8UC3)
      cv.CvtColor(mono_image, flow_image, cv.CV_GRAY2BGR)

      # for every region_size*region_size region in the image:
      for region_i in xrange(rows/region_size):
	 for region_j in xrange(cols/region_size):
	    # smooth over region to obtain velocity vectors
	    vx_accum = vy_accum = 0.
            for i in xrange(region_i*region_size,region_i*region_size+region_size):
	       for j in xrange(region_j*region_size,region_j*region_size+region_size):
		  x = self.velX[i,j]
		  y = self.velY[i,j]
		  vx_accum += x
		  vy_accum += y
	    vx_avg = vx_accum / (region_size*region_size)
	    vy_avg = vy_accum / (region_size*region_size)
	    # draw an arrow (just a line)
	    arr_center = (acx, acy) = (region_j*region_size+region_size/2,
		  region_i*region_size+region_size/2)
	    mult=10
	    cv.Line(flow_image, arr_center, (acx+vx_avg*mult,acy+vy_avg*mult), (0,0,255))
      cv.ShowImage('flow', flow_image)
      cv.WaitKey(3)


   def pcl_callback(self, pointcloud_msg):
      return
      rospy.loginfo('Received pointcloud message')
      # TODO
      bridge = CvBridge()
      try:
         timestamp = pointcloud_msg.header.stamp
	 #depth_img = bridge.imgmsg_to_cv(img_message)
	 if self.lastPointCloud != None and self.velX != None and self.velY != None:
	    # do the depth change estimate
	    #dZ = getDepthChanges(self.lastPointCloud, pointcloud_msg, self.velX, self.velY)
	    self.calc_3d_vel_estimate(self.lastPointCloud, pointcloud_msg)
	    self.lastPointCloud = pointcloud_msg
	    self.publish_markers(64)
	 else:
	    self.lastPointCloud = pointcloud_msg
      except CvBridgeError, e:
	 rospy.logerr(e)


   def calc_3d_vel_estimate(self, pcOld, pcNew):
      (rows, cols) = (self.velX.rows, self.velY.cols)
      self.rows = rows
      self.cols = cols

      # initialize velocity matrix (2d array of 3-tuples)
      V = [[None]*cols for row in xrange(rows)]

      # iterate through the point clouds
      self.old_points = [pt for pt in point_cloud.read_points(pcOld)]
      self.new_points = [pt for pt in point_cloud.read_points(pcNew)]

      # get 3d velocity estimate at each pixel
      for i in xrange(rows):
	 for j in xrange(cols):
	    iDiff = int(round(self.velX[i,j]))
	    jDiff = int(round(self.velY[i,j]))

	    iOld = i - iDiff
	    jOld = j - jDiff

	    # handle out of bounds: set V at (i,j) to None
	    # TODO think of a better way
	    if iOld<0 or iOld >=rows or jOld<0 or jOld>cols:
	       V[i][j] = None
	    else:
	       new_point = self.new_points[rows*i + j]
	       old_point = self.old_points[rows*iOld + jOld]

	       # use difference b/w new_point and old_point as vel estimate
	       V[i][j] = (vel_x, vel_y, vel_z) = tuple(new-old for (new,old) in zip(new_point,old_point))

      self.vel3d = V

   def publish_markers(self, region_size):
      # region_size: size in pixels to average over (square) - width and height must divide into the value
      # TODO refactor this, it does more that publish markers, it does smoothing
      # TODO smooth using gaussian distributions?

      marker_array = MarkerArray()
      mult_factor = 10

      # for every region_size*region_size region in the depth image:
      for region_i in xrange(self.rows/region_size):
	 for region_j in xrange(self.cols/region_size):
	    # smooth over region to obtain velocity and position vectors
	    vector_count = 0.
	    vx_accum = vy_accum = vz_accum = 0.
	    px_accum = py_accum= pz_accum= 0.
            for i in xrange(region_i*region_size,region_i*region_size+region_size):
	       for j in xrange(region_j*region_size,region_j*region_size+region_size):
		  vector = self.vel3d[i][j]
		  if vector != None:
		     (vx, vy, vz) = vector
		     if not (math.isnan(vx) or math.isnan(vy) or math.isnan(vz)):
			vector_count += 1
			vx_accum += vx  * mult_factor
			vy_accum += vy * mult_factor
			vz_accum += vz * mult_factor
			(px, py, pz) = self.new_points[i * self.rows + j]
			px_accum += px
			py_accum += py
			pz_accum += pz
	    if vector_count > 0:	     
	       avg_vel = (vx_accum/vector_count, vy_accum/vector_count, vz_accum/vector_count)
	       avg_pos = (px_accum/vector_count, py_accum/vector_count, pz_accum/vector_count)
	       # make an arrow visualization Marker
	       marker = Marker()
	       marker.header.frame_id = "/openni_depth_optical_frame"
	       marker.type = marker.ARROW
	       marker.action = marker.ADD
	       marker.scale.x = avg_vel[0]
	       marker.scale.y = avg_vel[1]
	       marker.scale.z = avg_vel[2]
	       marker.color.a = 1.0
	       marker.color.r = 1.0
	       marker.color.g = 0.0
	       marker.color.b = 0.0
	       marker.pose.orientation.w = 1.0
	       marker.pose.position.x = avg_pos[0]
	       marker.pose.position.y = avg_pos[1]
	       marker.pose.position.z = avg_pos[2]
	       
	       marker.id = region_i * self.rows + region_j
	       marker_array.markers.append(marker)
	    
      # publish MarkerArray
      self.marker_pub.publish(marker_array)


   # Given point cloud messages pclOld and pclNew, optical flow
   # components X and Y, give the Z flow estimate
   # TODO OBSOLETE DELETE THIS
   def getDepthChanges(self, pclOld, pclNew, X, Y):
      rospy.loginfo("calculating depth change")
      Z = cv.CreateMat(X.rows, X.cols, X.type) # X.type?
      self.old_depth = [pt[2] for pt in point_cloud.read_points(pclOld)]
      self.new_depth = [pt[2] for pt in point_cloud.read_points(pclNew)]

      (rows, cols) = (X.rows, X.cols)

      for i in xrange(rows):
	 for j in xrange(cols):
	    iDiff = round(X[i,j])
	    jDiff = round(Y[i,j])
	    iOld = i - iDiff
	    jOld = j - jDiff
	    # handle out of bounds TODO think of a better way
	    if iOld<0 or iOld >=rows or jOld<0 or jOld>cols:
	       Z[i,j] = float("nan")
	    else:
	       Z[i,j] = self.new_depth[rows*i + j] - self.old_depth[rows*iOld + jOld]

      self.velZ = Z



def main(args):
   node_name = "depth_change_estimator"
   DepthChangeEstimator(node_name)

   rospy.spin()
   cv.DestroyAllWindows()
   rospy.loginfo('node ' + node_name + ' terminated')

if __name__ == '__main__':
    main(sys.argv)
