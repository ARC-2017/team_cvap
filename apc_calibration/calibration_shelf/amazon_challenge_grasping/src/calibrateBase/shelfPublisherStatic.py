#! /usr/bin/python

from baseScanStatic import *

rospy.init_node('shelf_publisher_static')
bs_static = baseScanStatic()
bs_static.estimateShelfFrame()
