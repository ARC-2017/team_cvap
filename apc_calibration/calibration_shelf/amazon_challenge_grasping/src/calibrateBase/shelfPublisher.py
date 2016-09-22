#! /usr/bin/python

from baseScan import *

rospy.init_node('shelf_publisher')
bs = baseScan()
bs.estimateShelfFrame()
