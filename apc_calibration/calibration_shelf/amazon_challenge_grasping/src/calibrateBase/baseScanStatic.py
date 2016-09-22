#! /usr/bin/python

import rospy
import tf
import numpy
import time
from numpy import linalg as LA
from math import *
from sensor_msgs.msg import LaserScan, JointState
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import math
from geometry_msgs.msg import PoseStamped
from select import select
import sys
import pickle
from collections import namedtuple
import rospkg
from grasping.myTypes import *
import random
from std_srvs.srv import Empty, EmptyResponse
import PyKDL as kdl


class baseScanStatic:
    def __init__(self, verbose=False):
        self.rangeData = LaserScan()
        rospy.sleep(1)
        self.listener = tf.TransformListener()
        self.laser_projector = LaserProjection()
        self.pc = []
        self.shelf_width = 0.875
        self.leg1 = []
        self.leg2 = []
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(10.0)
        self.rate_measurements = rospy.Rate(0.5)
        self.priorOri_in_base = []
        self.priorLeft_in_base = []
        self.priorRight_in_base = []
        self.odomL = []
        self.odomR = []
        self.priorAvailable = False
        self.newObsWeight = 0.2
        self.offsetXY = [0.0, 0.0]
        self.binOffset = 0.02
        self.reCalibrationCount = 4
        self.tolerance = 0.1
        self.updateRounds = 100
        self.asyncRate = 20
        self.limitInitX = True
        self.xLimit = 0.1
        self.rp = rospkg.RosPack()
        self.writeFile = False
        self.calculateTF = False
        self.sendTF = False
        self.readDefaultTF = False
        self._ready = False
        self.radius_th_leg = 0.1
        self.n_samples = 5
        self.sleep_between_scans = 2
        while not rospy.is_shutdown():
            try:
                self._shelfHeight = rospy.get_param("/apc/shelf_calibration/height")
                self._shelfRoll = rospy.get_param("/apc/shelf_calibration/roll")
                self._shelfPitch = rospy.get_param("/apc/shelf_calibration/pitch")
                break
            except:
                rospy.sleep(1.0)
                continue
        self.start_srv = rospy.Service(rospy.get_name() + '/calibrate', Empty, self.start_srv_callback)

    def start_srv_callback(self, req):
        self.readDefaultTF = True
        rospy.logwarn('Shelf static publisher callback')
        return EmptyResponse()

    def estimateShelfFrame(self):

        while not rospy.is_shutdown():
            self.rate.sleep()
            t_init = rospy.Time.now()

            if self.readDefaultTF:
                #rospy.logerr("Reading default shelf position xy from file, orientation assumed 0 yaw and roll pitch given from the launch")
                rootDir= rospkg.RosPack().get_path('calibration_data') + '/extrinsics/'
                file_shelf_front_default = rootDir + 'shelf_front_default.txt'
                array_shelf_front_default = numpy.loadtxt(file_shelf_front_default)
                shelfOri = [array_shelf_front_default[0], array_shelf_front_default[1]]
                shelfRot = 0.0
                legs = numpy.zeros((2, 2))
                legs[0][0] = shelfOri[0]
                legs[0][1] = shelfOri[1] + 0.5*self.shelf_width
                legs[1][0] = shelfOri[0]
                legs[1][1] = shelfOri[1] - 0.5*self.shelf_width
                self.sendTF = True

            if self.sendTF:

                self.br.sendTransform((shelfOri[0], shelfOri[1], self._shelfHeight),
                             tf.transformations.quaternion_from_euler(self._shelfRoll, self._shelfPitch, shelfRot),
                             rospy.Time.now(),
                             "/shelf_front",     # child
                             "/base"             # parent
                             )

                self.br.sendTransform((legs[0][0], legs[0][1], self._shelfHeight), \
                             tf.transformations.quaternion_from_euler(self._shelfRoll, self._shelfPitch, shelfRot),
                             rospy.Time.now(),
                             "/left_leg",        # child
                             "/base"             # parent
                             )

                self.br.sendTransform((legs[1][0], legs[1][1], self._shelfHeight), \
                             tf.transformations.quaternion_from_euler(self._shelfRoll, self._shelfPitch, shelfRot),
                             rospy.Time.now(),
                             "/right_leg",       # child
                             "/base"             # parent
                             )

