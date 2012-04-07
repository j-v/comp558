#!/usr/bin/env python
import sys
import roslib; roslib.load_manifest('comp558')
import rospy
import rosbag
from threading import Condition
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from IPython.Debugger import Tracer; debug_here = Tracer()

class Depth2PointsNode:
   def __init__(self, in_bag_filename, out_bag_filename,
	 depth_topic='/camera/depth/image', 
	 pointcloud_topic= '/camera/depth/points',
	 camerainfo_topic= '/camera/depth/camera_info',
	 nodelet_depth_topic = '/image_rect', # TODO image isn't actually rectified
	 nodelet_pointcloud_topic = '/points',
	 nodelet_camerainfo_topic = '/camera_info',
	 node_name='depth2points'):
      # ROS node initialization
      rospy.init_node(node_name)
      #rospy.on_shutdown(self.node_shutdown) # TODO make this work
      self.camerainfo_topic = camerainfo_topic
      self.pointcloud_topic = pointcloud_topic
      self.depth_topic = depth_topic
      self.depth_pub = rospy.Publisher(nodelet_depth_topic, Image)
      self.camerainfo_pub = rospy.Publisher(nodelet_camerainfo_topic, CameraInfo)
      self.points_sub = rospy.Subscriber(nodelet_pointcloud_topic, PointCloud2, self.points_callback)
      self.got_points_condition = Condition()
      
      rospy.loginfo(node_name + 'node initialized')
      rospy.loginfo('Loading bagfile:' + in_bag_filename)
      self.in_bag = rosbag.Bag(in_bag_filename)
      self.out_bag = rosbag.Bag(out_bag_filename, 'w')
      self.readbag()

   def readbag(self):
     self.got_points_condition.acquire()
     self.camera_info_msg = None
     camerainfo_published = False
     for topic, dmsg, t in self.in_bag.read_messages( \
	   topics=[self.depth_topic,self.camerainfo_topic]):
	if topic == self.camerainfo_topic:
	   self.camerainfo_pub.publish(dmsg)
	   camerainfo_published = True
	   
	else:
	   if camerainfo_published == False:
	      continue
	   self.pointcloud_received = False

	   self.depth_t = t # used for giving timestamp to newly created point cloud message
	   self.depth_stamp = dmsg.header.stamp
	   self.depth_pub.publish(dmsg)
	   print 'published depth image, waiting for pointcloud'
	   # wait til received point cloud message from subscriber
	   # we use the depth_image_proc/point_cloud_xyz nodelet for this
	   while not self.pointcloud_received:
	      self.got_points_condition.wait()
     self.got_points_condition.release()
     rospy.signal_shutdown('Bagfile completed read')
	

   def points_callback(self, msg):
     print 'got pointcloud'
     self.got_points_condition.acquire()
     # write the pointcloud message to out bagfile
     msg.header.stamp = self.depth_stamp
     self.out_bag.write(self.pointcloud_topic, msg, self.depth_t)
     self.pointcloud_received = True
     self.got_points_condition.notify()
     self.got_points_condition.release()

   def node_shutdown(self):
     rospy.loginfo('Closing bagfiles...')
     self.in_bag.close()
     self.out_bag.close()
     rospy.loginfo('Bagfiles closed')

def print_usage():
   print '''Convert a depth images in a bagfile to a pointcloud bag file.
Usage: depth2pointcloud_bag.py IN_BAG_FILENAME OUT_BAG_FILENAME'''

def main(args):
   print 'hellloooooo!!!!!!!'
   try:
      inbagfile = args[1]
      outbagfile = args[2]
   except Exception,e:
      print e
      print_usage()
      exit()
   Depth2PointsNode(inbagfile, outbagfile)
   rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
