#!/usr/bin/env python
import roslib; roslib.load_manifest('comp558')
import rospy
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import cv
import math
import sys
from comp558 import point_cloud
from itertools import izip
import numpy as np
from IPython.Debugger import Tracer; debug_here = Tracer()
from tf.transformations import quaternion_from_euler

'''
THIS IS NOT FOR TIME, AND ASSUMES A SPECIALLY PROCESSED BAG FILE
'''
class VelEstimator3d:
   def __init__(self, bag_filename, topics, msg_range=None):
      self.bag_filename = bag_filename
      self.topics = topics
      rospy.init_node('vel_3d')
      self.last_mono_image = None
      self.last_pointcloud = None
      self.cv_bridge = CvBridge()
      self.flow_window_size = (5,5)
      self.region_size = 64
      self.marker_pub = rospy.Publisher('/depth_change_marker_array', MarkerArray)
      self.pointcloud_pub = rospy.Publisher('/point_cloud', PointCloud2)
      cv.NamedWindow('flow')
      self.flow_2d_history = [] # smoothed
      self.vel_3d_history =[]

      self.do_flow_estimate(msg_range)
      while raw_input("visualize data?").lower() == 'y':
         self.visualize_preprocessed(msg_range)
      

   """ THE MAIN METHOD """
   def do_flow_estimate(self, msg_range=None):
      flowX = cv.CreateMat(480,640, cv.CV_32FC1)
      flowY = cv.CreateMat(480,640, cv.CV_32FC1)
      with rosbag.Bag(self.bag_filename) as bag:
	 triple = []
	 count = 0
	 for topic, msg, t in bag.read_messages(topics=self.topics):
	    # get triple of (depth, rgb, points) messages
	    triple.append(msg)
	    if len(triple) == 3:
	       if msg_range != None: 
		  if count >= msg_range[1]: break
		  count += 1
		  if count < msg_range[0]: continue
	       else: count += 1
	       print 'got message tuple', count
	       msg_tuple = (depth_msg, rgb_msg, points_msg) = tuple(triple)
	       mono_image = self.cv_bridge.imgmsg_to_cv(rgb_msg, "mono8")
	       if self.last_mono_image != None:
		  print 'calculating optical flow'
	          # 1. Do optical flow estimate, get flowX & flowY
		  cv.CalcOpticalFlowLK(self.last_mono_image, mono_image, self.flow_window_size,
			flowX, flowY)

		  # 2. Smooth over optical flow (grid regions in 2D image), 
		  # get flowX_smoothed & flowY_smoothed
		  print 'smoothing 2d optical flow'
		  flow_smoothed = (flowX_smoothed, flowY_smoothed) = self.smooth_flow(flowX, flowY, self.region_size)
		  self.flow_2d_history.append( flow_smoothed )

		  # 3. Do 3d velocity estimate given the 2d estimates and pointcloud messages
		  print 'estimating 3d velocities'
		  pixel_vels = self.estimate_3d_vel(flowX, flowY, self.last_points_msg, points_msg)
		  print 'smoothing 3d velocity estimate'
		  vel_smoothed = points, velocities = self.smooth_3d_vel(pixel_vels, points_msg, self.region_size)
		  self.vel_3d_history.append( vel_smoothed )


	       self.last_mono_image = mono_image
	       self.last_points_msg = points_msg
	       


	       triple = []


   ''' visualize the 2d and 3d velocity field estimates '''
   def visualize_preprocessed(self, msg_range=None):
      count = -1
      with rosbag.Bag(self.bag_filename) as bag:
	 triple = []
	 for topic, msg, t in bag.read_messages(topics=self.topics):
	    triple.append(msg)
	    if len(triple) == 3:
               if msg_range != None: 
		  if count >= msg_range[1]-1: break
		  count += 1
		  if count < msg_range[0]: 
		     triple = []
		     continue
	       else: count += 1
	       print 'got message tuple', count

	       # ignore first triple
	       if count == 1: 
		  triple = []
		  continue
	       msg_tuple = (depth_msg, rgb_msg, points_msg) = tuple(triple)
	       mono_image = self.cv_bridge.imgmsg_to_cv(rgb_msg, "mono8")
	       self.pointcloud_pub.publish(points_msg)
	       flow_x, flow_y = self.flow_2d_history[count-1]
	       points, vels = self.vel_3d_history[count-1]
	       self.draw_flow_2d(flow_x, flow_y, mono_image, 10.)
	       self.plot_3d_vel(points, vels, 1.)
	       cv.WaitKey(3) # will this help?
	       raw_input("press ENTER")
	       triple = []

	 print "End of data"
	       
	       
	       

   def plot_3d_vel(self, p_arr, v_arr, v_scale=1.0):

      marker_array = MarkerArray()

      for i in xrange(p_arr.size):
	 #debug_here()
	 p = p_arr[i]
	 v = vx,vy,vz = v_arr[i]
	 marker = Marker()
	 marker.header.frame_id = "/openni_depth_optical_frame"
	 marker.type = marker.ARROW
	 marker.action = marker.ADD
	 marker.scale.x = 1.0
	 marker.scale.y = 1.0
	 marker.scale.z = math.sqrt(vx*vx + vy*vy + vz*vz) * v_scale # z is length of arrow
	 marker.color.a = 1.0
	 marker.color.r = 1.0
	 marker.color.g = 0.0
	 marker.color.b = 0.0
	 pitch = math.atan2(v[1], math.sqrt(v[0]*v[0]+v[2]*v[2]))
	 yaw = math.atan2(v[0],v[2])
	 (w,x,y,z) = quaternion_from_euler(pitch,yaw,0)
	 marker.pose.orientation.x = x
	 marker.pose.orientation.y = y
	 marker.pose.orientation.z = z
	 marker.pose.orientation.w = w
	 marker.pose.position.x = p[0]
	 marker.pose.position.y = p[1]
	 marker.pose.position.z = p[2]
	 marker.id = i

	 marker_array.markers.append(marker)
      #debug_here()
      self.marker_pub.publish(marker_array)


	 
      
   def smooth_3d_vel(self, pixel_vels, pointcloud_message, region_size):
      rows, cols = pixel_vels.shape 
      s_rows, s_cols = rows/region_size,cols/region_size
      points = [pt for pt in point_cloud.read_points(pointcloud_message)]
      result_points = np.zeros(s_rows*s_cols, dtype=('f4,f4,f4'))
      velocities = np.zeros(s_rows*s_cols, dtype=('f4,f4,f4'))

      isnan = math.isnan
      point_index = 0
      for region_i in xrange(s_rows):
	 for region_j in xrange(s_cols):
	    v_count = 0.
	    v_accum = np.array([0.,0.,0.])
	    p_accum = np.array([0., 0., 0.])
            for i in xrange(region_i*region_size,region_i*region_size+region_size):
	       for j in xrange(region_j*region_size,region_j*region_size+region_size):
		  v = pixel_vels[i,j]
		  vx, vy, vz = v
		  v = np.array((vx,vy,vz))
		  if not (isnan(vx) or isnan(vy) or isnan(vz)):
		     p = px,py,pz = np.array(points[i * cols + j]) 
		     if not (isnan(px) or isnan(py) or isnan(pz)):
			p_accum += p
		        v_accum += v
			v_count += 1
	    if not (v_count > 0): v_count = 1. # prevent division by zero
	    result_points[point_index] = tuple(p_accum / v_count)
	    velocities[point_index] = tuple(v_accum / v_count)
	    point_index += 1

      return result_points, velocities
      

   def estimate_3d_vel(self, flowX, flowY, last_points_msg, points_msg):
      rows, cols = flowX.rows, flowX.cols
      last_points = [pt for pt in point_cloud.read_points(last_points_msg)]
      points = [pt for pt in point_cloud.read_points(points_msg)]

      pixel_vels_3d = np.empty([rows,cols],dtype=('f4,f4,f4'))
      
      # get 3d velocity estimate at each pixel
      for i in xrange(rows):
	 for j in xrange(cols):
	    iDiff = int(round(flowX[i,j]))
	    jDiff = int(round(flowY[i,j]))
	    iOld = i - iDiff
	    jOld = j - jDiff

	    # handle out of bounds: set V at (i,j) to nans
	    # TODO think of a better way?
	    if iOld<0 or iOld >=rows or jOld<0 or jOld>=cols:
	       try:
	          pixel_vels_3d[i,j] = (np.nan, np.nan, np.nan)
	       except:
		  import pdb; pdb.set_trace()
	    else:
	       new_point = np.array(points[cols*i + j]) 
	       old_point = np.array(last_points[cols*iOld + jOld])

	       # use difference b/w new_point and old_point as vel estimate
	       pixel_vels_3d[i,j] = (vel_x, vel_y, vel_z) = tuple(new_point - old_point)


      return pixel_vels_3d
      


   def smooth_flow(self, flowX, flowY, region_size):
      rows, cols = flowX.rows, flowX.cols
      s_rows, s_cols = rows/region_size,cols/region_size
      flowX_smoothed = np.empty([s_rows,s_cols])
      flowY_smoothed = np.empty([s_rows,s_cols])
      region_sqr = region_size * region_size

      for region_i in xrange(s_rows):
	 for region_j in xrange(s_cols):
	    vx_accum = vy_accum = 0.
            for i in xrange(region_i*region_size,region_i*region_size+region_size):
	       for j in xrange(region_j*region_size,region_j*region_size+region_size):
		  x = flowX[i,j]
		  y = flowY[i,j]
		  vx_accum += x
		  vy_accum += y
	    flowX_smoothed[region_i,region_j] = vx_accum / region_sqr 
	    flowY_smoothed[region_i,region_j] = vy_accum / region_sqr

      return (flowX_smoothed, flowY_smoothed)

   def draw_flow_2d(self, flowX_smoothed, flowY_smoothed, mono_image, scale=1):
      rows, cols = mono_image.rows, mono_image.cols
      s_rows, s_cols = flowY_smoothed.shape
      flow_image = cv.CreateMat(rows, cols, cv.CV_8UC3)
      cv.CvtColor(mono_image, flow_image, cv.CV_GRAY2BGR)
      region_size = rows/s_rows

      for region_i in xrange(s_rows):
	 for region_j in xrange(s_cols):
	    #draw flow arrow
	    arr_center = (acx, acy) = (region_j*region_size+region_size/2,
		  region_i*region_size+region_size/2)
	    try:
	       if not math.isnan(flowX_smoothed[region_i,region_j]) \
	        and not math.isnan(flowY_smoothed[region_i,region_j]):
		  arr_x = int(acx+flowX_smoothed[region_i,region_j]*scale)
		  arr_y = int(acy+flowY_smoothed[region_i,region_j]*scale)
		  arr_x = max(min(arr_x, sys.maxint), -sys.maxint)
		  arr_y = max(min(arr_y, sys.maxint), -sys.maxint)
		  cv.Line(flow_image, arr_center, (arr_x, arr_y), (0,0,255))
	    except Exception,e:
	       print e
	       #import pdb; pdb.set_trace()
      cv.ShowImage('flow', flow_image)
      cv.WaitKey(3)



      
      pass



def main(args):
   topics = [ #we expect topics in this order
      '/camera/depth/image',
      '/camera/rgb/image_color',
      '/camera/depth/points'
      ]
   bag_filename = args[1]
   msg_range = (0,6)
   VelEstimator3d(bag_filename, topics, msg_range)
   rospy.spin()
   cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
