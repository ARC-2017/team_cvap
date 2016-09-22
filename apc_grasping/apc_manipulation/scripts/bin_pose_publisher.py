#!/usr/bin/python

#   bin_pose_publisher.py
#
#   Created on: June 18, 2016
#   Authors:   Francisco Vina
#              fevb@kth.se
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


import rospy
import tf
import yaml
import argparse
import tf
import PyKDL as kdl
from tf_conversions import posemath

class BinPosesPublisher():

    def __init__(self, bin_poses_filename):
        self.read_poses(bin_poses_filename)
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._rate = rospy.Rate(10.0)
        self._tf_listener = tf.TransformListener()
        self._F_left_gripper_base_left_gripper = kdl.Frame()
        self._F_right_gripper_base_right_gripper = kdl.Frame()

        self._got_frames = False


    def read_poses(self, bin_poses_filename):
        if bin_poses_filename is not None:
            try:
                bin_poses_file = open(bin_poses_filename, 'r')
                self._bin_poses = yaml.load(bin_poses_file)
                bin_poses_file.close()
            except IOError as err:
                rospy.logerr('Could not load bin poses from file ' + bin_poses_filename)

        self._bin_poses_tf = []

        for arm in self._bin_poses:
            for pose_name in self._bin_poses[arm]:
                bin_pose_id = pose_name.strip('_pose') + '_' + arm
                bin = pose_name[0:5]

                pose = self._bin_poses[arm][pose_name]
                orientation = pose['orientation']
                position = pose['position']

                rotation= [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
                translation= [position['x'], position['y'], position['z']]

                self._bin_poses_tf.append({})
                self._bin_poses_tf[-1]['bin_pose_id'] = bin_pose_id
                self._bin_poses_tf[-1]['bin'] = bin
                self._bin_poses_tf[-1]['translation'] = translation
                self._bin_poses_tf[-1]['rotation'] = rotation

    def get_gripper_frames(self):

        while not rospy.is_shutdown() and not self._got_frames:

            try:
                self._F_left_gripper_base_left_gripper = posemath.fromTf(self._tf_listener.lookupTransform('left_gripper_base',
                                                                                                           'left_gripper',
                                                                                                           rospy.Time(0)))

                self._F_right_gripper_base_right_gripper = posemath.fromTf(self._tf_listener.lookupTransform('right_gripper_base',
                                                                                                           'right_gripper',
                                                                                                           rospy.Time(0)))

                self._got_frames = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('[bin_pose_publisher]: haven\'t received gripper frames')
                rospy.sleep(1.0)
                continue

    def publish_poses(self):

        while not rospy.is_shutdown():

            try:
                self._F_left_gripper_base_left_gripper = posemath.fromTf(self._tf_listener.lookupTransform('left_gripper_base',
                                                                                                           'left_gripper',
                                                                                                           rospy.Time(0)))

                self._F_right_gripper_base_right_gripper = posemath.fromTf(self._tf_listener.lookupTransform('right_gripper_base',
                                                                                                           'right_gripper',
                                                                                                           rospy.Time(0)))


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('[bin_pose_publisher]: haven\'t received gripper frames')
                rospy.sleep(1.0)
                continue

            for pose in self._bin_poses_tf:
                F  = posemath.fromTf((tuple(pose['translation']), tuple(pose['rotation'])))
                # left arm
                if pose['bin_pose_id'].rfind('left_arm') > 0:
                    translation, rotation = posemath.toTf(F*self._F_left_gripper_base_left_gripper)
                # right arm
                else:
                    translation, rotation = posemath.toTf(F*self._F_right_gripper_base_right_gripper)

                self._tf_broadcaster.sendTransform(translation=translation,
                                                   rotation=rotation,
                                                   time=rospy.Time.now(),
                                                   child=pose['bin_pose_id'],
                                                   parent=pose['bin'])
            self._rate.sleep()


if __name__=='__main__':

    rospy.init_node('bin_pose_publisher')
    parser = argparse.ArgumentParser(description='Publish in TF bin pose file')
    parser.add_argument('bin_poses_filepath',type=str,
                        help='Filepath with bin poses.')

    args = parser.parse_args(rospy.myargv()[1:])
    bin_poses_pub = BinPosesPublisher(args.bin_poses_filepath)
    rospy.sleep(10.0)
    print('here.....')
    bin_poses_pub.get_gripper_frames()
    bin_poses_pub.publish_poses()
