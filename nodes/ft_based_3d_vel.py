#!/usr/bin/env python
import roslib; roslib.load_manifest('comp558')
import rospy
import rosbag
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import cv
import math
import sys
from comp558 import point_cloud, flowlogger
from itertools import izip
import numpy as np
from IPython.Debugger import Tracer; debug_here = Tracer()
from tf.transformations import quaternion_from_euler
from time import time
# TODO lingering markers
'''
THIS IS NOT REALTIME, AND ASSUMES A SPECIALLY PROCESSED BAG FILE
First it reads the bag file, which consists of triples of depth, rgb, and pointcloud messages, frame by frame. It performs a 2d optical flow es
'''
class VelEstimator3d:
   def __init__(self, bag_filename, topics, msg_range=None, log_filename=None, playback_only=False):
      self.bag_filename = bag_filename
      self.log_filename = log_filename
      if self.log_filename == None: self.log_filename = bag_filename

	
      self.max_features = 40
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

      if not playback_only:
	 if raw_input('this may overwrite data. continue? [y/n]').lower() == 'y':
	    self.flow_logger = flowlogger.Flow2dLogger(self.log_filename + '.flow2d','w')
	    self.vel_logger = flowlogger.Vel3dLogger(self.log_filename + '.vel3d','w')
	    self.do_flow_estimate(msg_range)
      while raw_input("visualize data?").lower() == 'y':
         self.visualize_preprocessed(msg_range)
      

   """ THE MAIN METHOD """
   def do_flow_estimate(self, msg_range=None):
      # initialize optical flow parameters
      #flowX = cv.CreateMat(480,640, cv.CV_32FC1)
      #flowY = cv.CreateMat(480,640, cv.CV_32FC1)
      pyr1 = cv.CreateImage((480,640),cv.IPL_DEPTH_8U,1)
      pyr2 = cv.CreateImage((480,640),cv.IPL_DEPTH_8U,1)    # pyramids required by the LK algo. Just space.
      winSize = (3,3) # defines the patch of window that we'll look at
      maxPyrUsed = 5 # defines the level of the pyramid
      stoppingCriteria = ( cv.CV_TERMCRIT_ITER | cv.CV_TERMCRIT_EPS, 100,0.03) #stop after 20 iterations or epsilon is better than 0.3
      dontPrecomputePyrForNextFrame = 1
      self.max_features = 40
      eig_image = cv.CreateImage((480,640),cv.IPL_DEPTH_32F,1)    # space required by the feature tracking
      temp_image = cv.CreateImage((480,640),cv.IPL_DEPTH_32F,1)   # space required by the feature tracking
      feature_quality_level = 0.01
      feature_min_distance = 5

      
      with rosbag.Bag(self.bag_filename) as bag:
	 triple = []
	 count = 0
	 for topic, msg, t in bag.read_messages(topics=self.topics):
	    # get triple of (depth, rgb, points) messages
	    triple.append(msg)
	    if len(triple) == 3:
	       stamp = msg.header.stamp.to_nsec()
	       if msg_range != None: 
		  if count >= msg_range[1]: break
		  count += 1
		  if count < msg_range[0]: continue
	       else: count += 1
	       print 'got message tuple', count
	       msg_tuple = (depth_msg, rgb_msg, points_msg) = tuple(triple)
	       mono_image = self.cv_bridge.imgmsg_to_cv(rgb_msg, "mono8")
	       if self.last_mono_image != None:
		  total_start = time()

		  print 'calculating optical flow'
	          # 1. Do optical flow estimate, get flowX & flowY
		  stime = time()
		  # cv.CalcOpticalFlowLK(self.last_mono_image, mono_image, self.flow_window_size,
		  #                       flowX, flowY)
	          prevFrameFeatures = cv.GoodFeaturesToTrack(self.last_mono_image,
			eig_image, temp_image, numbFeaturesToDetect, feature_quality_level, feature_min_distance)
                  (currFrameFeatures,status,track_error) = cv.CalcOpticalFlowPyrLK( \
			self.last_mono_image, mono_image,
			pyr1, pyr2, prevFrameFeatures, winSize, maxPyrUsed,
			stoppingCriteria,
			dontPrecomputePyrForNextFrame)

		  print 'optical flow calculation:', time()-stime, 'sec'

		  # TODO obsolete code, we don't smooth on 2d optical flow w/ features.
		  # TODO we could consider using the err vector for logging, training
		  # 2. Smooth over optical flow (grid regions in 2D image), 
		  # get flowX_smoothed & flowY_smoothed
		  # print 'smoothing 2d optical flow'
		  # flow_smoothed = (flowX_smoothed, flowY_smoothed) = self.smooth_flow(flowX, flowY, self.region_size)
		  # self.flow_2d_history.append( flow_smoothed )
		  
		  # log smoothed 2d flow data
		  flowpoints = []
		  flowvels = []
		  for i in xrange(numbFeaturesToDetect):
		     if status[i] == 0: continue
		     else:
			px = prevFrameFeatures[i][0]
			py = prevFrameFeatures[i][1]
			qx = currFrameFeatures[i][0]
			qy = currFrameFeatures[i][1]
			flowpoints.append( (int(round(px)),int(round(py))) )
			flowvels.append( ( qx-px , qy-py) )
		  self.flow_logger.write(stamp, flowpoints, flowvels)

		  # 3. Do 3d velocity estimate given the 2d estimates and pointcloud messages
		  print 'estimating 3d velocities'
		  # TODO
		  points_3d, vels_3d = self.estimate_3d_vel(flowpoints, flowvels, self.last_points_msg, points_msg)
		  # log 3d velocity data
		  self.vel_logger.write(stamp, points_3d, vels_3d)

		  total_elapsed = time()-total_start
		  print 'Total processing:', total_elapsed, 'sec'

	       self.last_mono_image = mono_image
	       self.last_points_msg = points_msg
	       
	       triple = []


   ''' visualize the 2d and 3d velocity field estimates '''
   def visualize_preprocessed(self, msg_range=None):
      count = -1
      # open log files
      flowlog = flowlogger.Flow2dLogger(self.log_filename + '.flow2d')
      vellog = flowlogger.Vel3dLogger(self.log_filename + '.vel3d')
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

	       # read the messages to display
	       msg_tuple = (depth_msg, rgb_msg, points_msg) = tuple(triple)
	       mono_image = self.cv_bridge.imgmsg_to_cv(rgb_msg, "mono8")

	       # display point cloud
	       self.pointcloud_pub.publish(points_msg)
	       # read 3d velocity log
	       (t, points, vels) = vellog.read()
	       # plot 3d velocity data
	       self.plot_3d_vel(points, vels, 1.)

	       # read 2d optical flow log
	       (t, points, vels) = flowlog.read()
	       # draw optical flow lines onto the image
	       self.draw_flow_2d(points,vels,mono_image,10)

	       cv.WaitKey(3) # will this help?
	       raw_input("press ENTER")
	       triple = []

	 print "End of data"
	       
   def plot_3d_vel(self, p_arr, v_arr, v_scale=1.0):

      marker_array = MarkerArray()

      for i in xrange(len(p_arr)):
	 p = p_arr[i]
	 v = vx,vy,vz = v_arr[i]
	 marker = Marker()
	 marker.header.frame_id = "/openni_rgb_optical_frame"
	 marker.type = marker.ARROW
	 marker.action = marker.ADD
	 marker.color.a = 1.0
	 marker.color.r = 1.0
	 marker.color.g = 0.0
	 marker.color.b = 0.0
	 marker.points.append(Point(p[0],p[1],p[2]))
	 marker.points.append(Point(p[0]+vx*v_scale,p[1]+vy*v_scale,p[2]+vz*v_scale)) 
	 marker.scale.x=0.05
	 marker.scale.y=0.1
	 marker.id = i 

	 marker_array.markers.append(marker)
      
      for i in xrange(len(p_arr),self.max_features):
	 marker = Marker()
	 marker.action = marker.DELETE
	 marker.id = i
	 marker_array.markers.append(marker)


      self.marker_pub.publish(marker_array)


   # TODO obsolete?
   def smooth_3d_vel(self, pixel_vels, pointcloud_message, region_size):
      rows, cols = pixel_vels.shape 
      stime = time()
      points = [pt for pt in point_cloud.read_points(pointcloud_message)]
      print 'point cloud iteration: ', (time()-stime), 'sec'
      s_rows, s_cols = rows/region_size,cols/region_size
      result_points = []
      velocities = []

      stime = time()
      isnan = math.isnan
      point_index = 0
      for region_i in xrange(s_rows):
	 for region_j in xrange(s_cols):
	    v_count = 0.
	    v_accum = [0.,0.,0.]
	    p_accum = [0.,0.,0.]
            for i in xrange(region_i*region_size,region_i*region_size+region_size):
	       for j in xrange(region_j*region_size,region_j*region_size+region_size):
		  v = vx,vy,vz = pixel_vels[i,j]
		  if not (isnan(vx) or isnan(vy) or isnan(vz)):
		     p = px,py,pz = points[i * cols + j] 
		     if not (isnan(px) or isnan(py) or isnan(pz)):
			p_accum[0]+=px; p_accum[1]+=py; p_accum[2]+=pz
			v_accum[0]+=vx; v_accum[1]+=vy; p_accum[2]+=vz
			v_count += 1
	    if not (v_count > 0): v_count = 1. # prevent division by zero
	    result_points.append( (p_accum[0]/v_count, p_accum[1]/v_count, p_accum[2]/v_count) )
	    velocities.append( (v_accum[0]/v_count, v_accum[1]/v_count, v_accum[2]/v_count))

      print 'smoothing 3d velocity: ', (time() - stime), 'sec'
      return result_points, velocities
      

   def estimate_3d_vel(self, points_2d, vels_2d, last_points_msg, points_msg):
      stime = time()
      last_points = [pt for pt in point_cloud.read_points(last_points_msg)]
      elapsed = time()-stime
      print 'point cloud iteration: ', elapsed, 'sec'

      stime = time()
      points = [pt for pt in point_cloud.read_points(points_msg)]
      elapsed = time()-stime
      print 'point cloud iteration: ', elapsed, 'sec'
      
      points_3d = []
      vels_3d = []
      # get 3d velocity estimate at each pixel
      stime = time()
      for pt, vel in izip(points_2d, vels_2d):
      #for i in xrange(rows):
	 #for j in xrange(cols):
	    px, py = pt
	    vx, vy = vel

	    p2x, p2y = int(round(px + vx)), int(round(py + vy))

	    # handle out of bounds: set V at (i,j) to NaN 
	    # TODO think of a better way?
	    #if iOld<0 or iOld >=rows or jOld<0 or jOld>=cols:
	    # CORRECTION
	    if not (p2x<0 or p2x>=640 or p2y<0 or p2y>=480) :
	       newp = points[640*p2y+p2x]
	       oldp = last_points[640*py+px]

	       points_3d.append( oldp )
	       vels_3d.append(( newp[0]-oldp[0], newp[1]-oldp[1], newp[2]-oldp[2] ))
      elapsed = time()-stime
      print 'Feature-wise 3d velocity estimate: ', elapsed, 'sec'


      return points_3d, vels_3d
      


   # TODO Obsolete code, we want a feature based version
   def smooth_flow(self, flowX, flowY, region_size):
      stime = time()

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

      elapsed = time() - stime
      print 'Smoothing 2d flow: ', elapsed, 'sec'
      return (flowX_smoothed, flowY_smoothed)

   def draw_flow_2d(self, points, vels, mono_image, scale=1):
      stime = time()
      flow_image = cv.CreateMat(mono_image.rows, mono_image.cols, cv.CV_8UC3)
      cv.CvtColor(mono_image, flow_image, cv.CV_GRAY2BGR)

      for (pt, vel) in zip(points, vels):
	 (x, y)=pt
	 (vx, vy)=vel
	 arr_x = int(x+vx*scale)
	 arr_y = int(y+vy*scale)
	 arr_x = max(min(arr_x, sys.maxint), -sys.maxint)
	 arr_y = max(min(arr_y, sys.maxint), -sys.maxint)
	 cv.Line(flow_image, (x,y), (arr_x, arr_y), (0,0,255))

      cv.ShowImage('flow', flow_image)
      elapsed = time() - stime
      print 'drawing 2d flow: ', elapsed, 'sec'
      cv.WaitKey(3)


def main(args):
   topics = [ #we expect topics in this order
      '/camera/depth/image',
      '/camera/rgb/image_color',
      '/camera/depth/points'
      ]
   bag_filename = args[1]
   #msg_range = (0,6)
   msg_range = None
   playback_only=False
   VelEstimator3d(bag_filename, topics, msg_range, playback_only=playback_only)
   rospy.spin()
   cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
