#!/usr/bin/env python

#   calib_camera_mount
#
#   Created on: May 10, 2016
#   Authors:   Francisco Vina
#             fevb <at> kth.se
#

#  Copyright (c) 2016, Francisco Vina, CVAP, KTH
#    All rights reserved.

#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#       * Redistributions of source code must retain the above copyright
#         notice, this list of conditions and the following disclaimer.
#       * Redistributions in binary form must reproduce the above copyright
#         notice, this list of conditions and the following disclaimer in the
#         documentation and/or other materials provided with the distribution.
#       * Neither the name of KTH nor the
#         names of its contributors may be used to endorse or promote products
#         derived from this software without specific prior written permission.

#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
#    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
#    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
#    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
#    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# coding: utf-8
import rospy
import tf
import PyKDL as kdl
from tf_conversions import posemath
import numpy as np
import sys
import copy
import rospkg

class CameraMountCalibration:
    def __init__(self):
        
        self._tf_listener = tf.TransformListener()

    def calib(self):

        self._left_gripper_pose = kdl.Frame()
        self._right_gripper_pose = kdl.Frame()
        self._camera_mount_pose = kdl.Frame()

        while not rospy.is_shutdown():
            try:
                (trans, rot) = self._tf_listener.lookupTransform('/base', 'left_gripper', rospy.Time(0))
                self._left_gripper_pose = posemath.fromTf((trans, rot))

                (trans, rot) = self._tf_listener.lookupTransform('/base', 'right_gripper', rospy.Time(0))
                self._right_gripper_pose = posemath.fromTf((trans, rot))
                break
            except:
                rospy.logerr(rospy.get_name() + ': could not get left/right gripper poses for calibration of the camera mount')
                rospy.sleep(1.0)

        vl = copy.deepcopy(self._left_gripper_pose.p)
        vl[2] = 0.0

        vr = copy.deepcopy(self._right_gripper_pose.p)
        vr[2] = 0.0

        v = vr - vl

        # center point between the 2 tripods of the camera mount
        center = vl + 0.5*v

        # discard the information about height (which is to be measured separately)
        self._camera_mount_pose.p[0] = center[0]
        self._camera_mount_pose.p[1] = center[1]
        self._camera_mount_pose.p[2] = -0.93

        # calculate the angle of rotation along the Z axis
        rotz_angle = np.arctan(v[0]/-v[1])
        rotz_angle = rotz_angle - np.pi*0.5
        R = kdl.Rotation.RPY(0, 0, rotz_angle)

        self._camera_mount_pose.M = R

        return True

    def print_calib(self):

        rospy.loginfo('Left gripper pos: ' + str(self._left_gripper_pose.p))
        rospy.loginfo('Right gripper pos: ' + str(self._right_gripper_pose.p))
        rospy.loginfo('Camera Mount pose (x, y, z, r, p, y): ' + str(self._camera_mount_pose.p)+ ' | ' + str(self._camera_mount_pose.M.GetRPY()))
        rospy.loginfo('Camera Mount pose (x, y, z, quaternion): ' + str(self._camera_mount_pose.p)+ ' | '
                      + str(self._camera_mount_pose.M.GetQuaternion()))

    def get_filepath(self):
        '''
        gets filepath to calibration file
        @return: file path (str)
        '''

        rospack = rospkg.RosPack()
        path = rospack.get_path('calibration_data')
        path = path + '/extrinsics'

        return path

    def save_calib(self):
        '''
        Saves calibration to extrinsic calib file in data package
        @return:
        '''

        path = self.get_filepath()
        file = path + '/camera_mount.txt'

        f = open(file, 'w')

        pose = ''

        for index in xrange(3):
            pose += str(self._camera_mount_pose.p[index]) + ' '

        q = self._camera_mount_pose.M.GetQuaternion()
        for index in xrange(4):
            pose += str(q[index]) + ' '

        f.write(pose)

        f.close()

        return





if __name__ == "__main__":

    rospy.init_node('camera_mount_calibration')

    camera_mount_calib = CameraMountCalibration()
    rospy.sleep(1.0)

    rospy.loginfo('Place left gripper on the left tripod at the mark indicated by the silver tape '
                  'and right gripper on the right tripod at the mark indicated by the silver tape.')

    raw_input("Press Enter when ready...")

    if camera_mount_calib.calib():
        camera_mount_calib.print_calib()
        camera_mount_calib.save_calib()