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


class baseScan:
    def __init__(self, verbose=False):
        self.rangeData = LaserScan()
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback_laser)
        rospy.sleep(1)
        self.listener = tf.TransformListener()
        self.laser_projector = LaserProjection()
        self.pc = []
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
        #self.pubShelfSep = rospy.Publisher('pubShelfSep', PoseStamped, queue_size=10)
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
        while not self._ready:
            rospy.logwarn('Shelf published waiting for scan callback')
            rospy.sleep(1.0)

        rospy.loginfo('[shelf_publisher]: starting shelf publisher')
        self.calculateTF = True
        return EmptyResponse()

    def callback_laser(self, data):
        self._ready = True
        self.rangeData = data

    def getCloud(self):
        cloud2 = self.laser_projector.projectLaser(self.rangeData)
        xyz = pc2.read_points(cloud2, skip_nans=True, field_names=("x", "y", "z"))
        self.pc = []
        while not rospy.is_shutdown():
            try:
                self.pc.append(xyz.next())
            except:
                break
        return self.pc

    def findLegsOnce(self):
        pc = self.getCloud()
        x = []
        y = []
        for i in range(len(pc)):
            x.append(pc[i][0])
            y.append(pc[i][1])
        radius = []

        if self.limitInitX:
            y = [y[i] for i in range(len(y)) if x[i] >= self.xLimit]
            x = [x[i] for i in range(len(x)) if x[i] >= self.xLimit]

        for i in range(len(x)):
            radius.append(math.sqrt(x[i]**2 + y[i]**2))
        n = radius.index(min(radius))

        x2 = [x[i] for i in range(len(x)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.7]
        y2 = [y[i] for i in range(len(y)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.7]
        radius2 = []

        for i in range(len(x2)):
            radius2.append(math.sqrt(x2[i]**2 + y2[i]**2))
        n2 = radius2.index(min(radius2))

        leg1_p = [x[n], y[n]]
        leg2_p = [x2[n2], y2[n2]]

        x1_avg = [x[i] for i in range(len(x)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) < self.radius_th_leg]
        y1_avg = [y[i] for i in range(len(y)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) < self.radius_th_leg]

        x2_avg = [x[i] for i in range(len(x)) if math.sqrt( (x[i]-x[n2])**2 + (y[i]-y[n2])**2 ) < self.radius_th_leg]
        y2_avg = [y[i] for i in range(len(y)) if math.sqrt( (x[i]-x[n2])**2 + (y[i]-y[n2])**2 ) < self.radius_th_leg]

        leg1 = [numpy.mean(x1_avg) + self.offsetXY[0], numpy.mean(y1_avg) + self.offsetXY[1]]
        leg2 = [numpy.mean(x2_avg) + self.offsetXY[0], numpy.mean(y2_avg) + self.offsetXY[1]]

        #leg1 = [leg1[0] + self.offsetXY[0], leg1[1] + self.offsetXY[1]]
        #leg2 = [leg2[0] + self.offsetXY[0], leg2[1] + self.offsetXY[1]]
        return [leg1, leg2] # left, right

    def findLegs(self):
        leg1_x = numpy.zeros(self.n_samples)
        leg1_y = numpy.zeros(self.n_samples)
        leg2_x = numpy.zeros(self.n_samples)
        leg2_y = numpy.zeros(self.n_samples)
        for i in range(0, self.n_samples):
            rospy.loginfo('taking laser scan: ')
            rospy.loginfo(i)
            legs_sample = self.findLegsOnce()
            leg1_x[i] = legs_sample[0][0]
            leg1_y[i] = legs_sample[0][1]
            leg2_x[i] = legs_sample[1][0]
            leg2_y[i] = legs_sample[1][1]
            time.sleep(self.sleep_between_scans)
            #self.rate_measurements.sleep()

        leg1_m = [numpy.mean(leg1_x), numpy.mean(leg1_y)]
        leg2_m = [numpy.mean(leg2_x), numpy.mean(leg2_y)]
        rospy.loginfo('done scan')
        rospy.loginfo(leg1_m)
        rospy.loginfo(leg2_m)

        legs = [leg1_m, leg2_m]
        if legs[0][1] < legs[1][1]:
            legs[0], legs[1] = legs[1], legs[0]
        self.leg1 = legs[0]
        self.leg2 = legs[1]
        return [self.leg1, self.leg2] # left, right

    def getShelfFrame(self):
        legs = self.findLegs()
        ori_x = (legs[0][0] + legs[1][0]) / 2.
        ori_y = (legs[0][1] + legs[1][1]) / 2.
        left_leg = legs[0]
        if legs[0][1] < legs[1][1]:
            left_leg = legs[1]
        rotAngle = atan2(ori_x - left_leg[0], left_leg[1] - ori_y)
        return [ori_x, ori_y], rotAngle, legs

    def tf2PoseStamped(self, xy, ori):
        shelfPoseMsg = PoseStamped()
        shelfPoseMsg.pose.position.x = xy[0]
        shelfPoseMsg.pose.position.y = xy[1]
        shelfPoseMsg.pose.position.z = 0.0
        shelfPoseMsg.pose.orientation.x = ori[0]
        shelfPoseMsg.pose.orientation.y = ori[1]
        shelfPoseMsg.pose.orientation.z = ori[2]
        shelfPoseMsg.pose.orientation.w = ori[3]
        shelfPoseMsg.header.frame_id = 'base'
        shelfPoseMsg.header.stamp = rospy.Time.now()
        return shelfPoseMsg

    def estimateShelfFrame(self):

        while not rospy.is_shutdown():
            self.rate.sleep()
            t_init = rospy.Time.now()

            if self.calculateTF:
                try:
                    shelfOri, shelfRot, legs = self.getShelfFrame()
                except:
                    rospy.logerr("Shelf calibration is not getting points")
                    continue

                self.calculateTF = False
                self.sendTF = True
                self.writeFile = True

            if self.sendTF:
                # F1 = kdl.Frame(kdl.Rotation.RPY(0, 0, shelfRot),
                #                     kdl.Vector(shelfOri[0], shelfOri[0], self._shelfHeight))
                # F2 = kdl.Frame(kdl.Rotation.RPY(self._shelfRoll, self._shelfPitch, 0), kdl.Vector(0,0,0))
                # F3 = F1*F2
                # position_xyz3 = F3.p
                # orientation_xyzw3 = F3.M.GetQuaternion()
                # self.br.sendTransform((position_xyz3[0], position_xyz3[1], position_xyz3[2]),
                #              orientation_xyzw3,
                #              rospy.Time.now(),
                #              "/shelf_front",     # child
                #              "/base"             # parent
                #              )

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

                #self.pubShelfSep.publish(self.tf2PoseStamped(shelfOri, tf.transformations.quaternion_from_euler(0, 0, shelfRot)))

            if self.writeFile:
                file_path = self.rp.get_path('calibration_data')
                f = open(file_path + "/extrinsics/shelf_front.txt", 'w')
                f.write(str(shelfOri[0])+'\t')
                f.write(str(shelfOri[1])+'\t')
                f.write(str(0.0)+'\t')
                quaternion_shelf = tf.transformations.quaternion_from_euler(0, 0, shelfRot)
                f.write(str(quaternion_shelf[0])+'\t')
                f.write(str(quaternion_shelf[1])+'\t')
                f.write(str(quaternion_shelf[2])+'\t')
                f.write(str(quaternion_shelf[3])+'\n')
                f.close()
                self.writeFile = False
