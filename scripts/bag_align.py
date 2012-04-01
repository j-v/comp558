#!/usr/bin/env python
import sys

import roslib; roslib.load_manifest('comp558')
import rospy, rosbag
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv

'''
bag_align.py
jvolkmar 2012
in: filename, list of topics, sync topic
out: bag file consisting of messages most aligned to sync topic, in order of the list of topics
'''

def print_usage():
   print sys.argv[0], 'in_file out_file sync_topic [topic1 topic2...]'
  
if len(sys.argv)<4:
   print_usage()
   exit(1)

in_filename = sys.argv[1]
out_filename = sys.argv[2]
sync_topic = sys.argv[3]
topics = [sync_topic]
for topic in sys.argv[4:]: topics.append(topic)

print 'Reading topics:'
for topic in topics: print topic
print 'in file', in_filename

# init list of times for each topic
msg_lists = {}
for topic in topics: msg_lists[topic] = []

inbag = rosbag.Bag(in_filename)

# read bag file and store all messages in memory
for topic,msg,t in inbag.read_messages(topics=topics):
   #stamp_nsec = msg.header.stamp.to_nsec()
   #msg_lists[topic].append(stamp_nsec)
   msg_lists[topic].append((msg, t))

# assume lists are sorted by msg.header.stamp

# find best matches, write to bag
with rosbag.Bag(out_filename, 'w') as outbag:
   for (syncmsg, synct) in msg_lists[sync_topic]:
      # write the message to sync to 
      outbag.write(sync_topic,syncmsg,synct)
      print 'wrote sync message (topic=%s)' % sync_topic
      for topic in topics[1:]:
	 sync_stamp_nano = syncmsg.header.stamp.to_nsec()
	 best_sync_diff = float("inf")
	 best_match = None
	 for (msg, t) in msg_lists[topic]:
	    stamp_nano = msg.header.stamp.to_nsec()
	    sync_diff = abs(sync_stamp_nano-stamp_nano)
	    if sync_diff<best_sync_diff:
	       best_sync_diff = sync_diff
	       best_match = (msg,t)
	 # found best match, write it
	 best_msg, best_t = best_match
	 #outbag.write(topic,best_msg,best_t)
	 outbag.write(topic,best_msg,synct) # have to use synct for rosbag to preserve write order
	 outbag.flush() # ??? not sure if its necessary
	 print 'wrote message (topic=%s)' % topic

print 'Finito.'
	 
