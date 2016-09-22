#!/usr/bin/python

#   TFCache.py
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


import tf
import PyKDL as kdl
from tf_conversions import posemath
import rospy
import rospkg
from baxter_interface.limb import Limb
import yaml
import random
import threading
import copy
import thread

class StaticTFCollector(threading.Thread):

    def __init__(self, missing_frames, tfs, rate, lock, tf_listener):
        '''

        @param missing_static_tfs: list of strings with missing static tfs
        @param static_tfs: dictionary of transforms (kdl.Frame) collected so far
        @param collect_tf_rate: rate (Hz) at which to collect transforms
        @param lock: RLock
        '''
        threading.Thread.__init__(self)
        self._missing_frames = missing_frames
        self._tfs = tfs
        self._rate = rospy.Rate(rate)
        self._lock = lock
        self._tf_listener = tf_listener


    def run(self):
        '''

        @return:
        '''

        self._collect_static_tfs()

    def _collect_static_tfs(self):

        while len(self._missing_frames) > 0 and not rospy.is_shutdown():
            self._lock.acquire()
            for frame in self._missing_frames:

                for trial in range(10):
                    if frame.rfind('gripper') > 0:
                        try:
                            (trans, rot) = self._tf_listener.lookupTransform(frame + '_base', frame, rospy.Time(0))
                            self._tfs[frame] = posemath.fromTf((trans, rot))
                            break

                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rospy.sleep(0.5)

                    else:
                        try:
                            (trans, rot) = self._tf_listener.lookupTransform('/base', frame, rospy.Time(0))
                            self._missing_frames.remove(frame)
                            self._tfs[frame] = posemath.fromTf((trans, rot))
                            break

                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rospy.sleep(0.5)

            self._lock.release()
            self._rate.sleep()

class TFCache():

    def __init__(self):

        self._tf_listener = tf.TransformListener()

        self._lock = threading.RLock()

        self._left_arm = Limb('left')
        self._right_arm = Limb('right')

        self._task = None
        self._static_tfs_param = None

        self._first_check = True

        while not rospy.is_shutdown():

            for trial in range(10):
                try:
                    self._task = rospy.get_param('/apc/task')
                    self._static_tfs_param = rospy.get_param('/apc/manipulation/static_tfs')
                    break
                except:
                    rospy.sleep(random.uniform(0, 1))
                    continue

            if self._task == None:
                rospy.logerr('[static_tf_cache]: could not get /apc/task parameter')

            elif self._static_tfs_param == None:
                rospy.logerr('[static_tf_cache]: could not get /apc/manipulation/static_tfs')

            else:
                break

        self._frames = []
        self._missing_frames = []
        self._get_static_tf_frame_names()

        # all transforms  w.r.t base frame except for transforms between (left/right)_gripper_base and (left/right)_gripper
        self._tfs = {}

        # start the collect static tf thread
        self._static_tf_collector_thread = StaticTFCollector(missing_frames=self._missing_frames,
                                                             tfs=self._tfs,
                                                             rate=1.0,
                                                             lock=self._lock,
                                                             tf_listener=self._tf_listener)

        self._static_tf_collector_thread.start()

        t = rospy.Timer(rospy.Duration(60.0), self._check_static_tfs)

    def _check_static_tfs(self, event):

        with self._lock:
            if len(self._missing_frames) > 0:
                grippers = ['left_gripper', 'right_gripper']
                for gripper in grippers:
                    if self._tfs.has_key(gripper):
                        if gripper in self._missing_frames:
                            self._missing_frames.remove(gripper)

                if len(self._missing_frames) > 0:
                    rospy.logerr('[static_tfs]: after 60 seconds could not find the following static tfs: ')

                    for frame in self._missing_frames:
                        rospy.logerr(frame)


    def _get_static_tf_frame_names(self):
        '''

        @return:
        '''
        for item in self._static_tfs_param:

            if item.has_key('filename'):
                if self._task == 'pick':
                    if item['filename'].rfind('pickBinPosesIDSTF') > 0 and item.has_key('package'):
                        pkg = rospkg.RosPack().get_path(item['package'])
                        filename = pkg + '/' + item['filename']
                        bin_poses = self._read_bin_poses_file(filename)

                        for bin_pose in bin_poses:
                            self._frames.append(bin_pose['bin_pose_id'])
                            self._missing_frames.append(bin_pose['bin_pose_id'])

            else:
                self._frames.append(item['child_frame_id'])
                self._missing_frames.append(item['child_frame_id'])

    def _read_bin_poses_file(self, bin_poses_filename):
        if bin_poses_filename is not None:
            try:
                bin_poses_file = open(bin_poses_filename, 'r')
                bin_poses = yaml.load(bin_poses_file)
                bin_poses_file.close()
            except IOError as err:
                rospy.logerr('Could not load bin poses from file ' + bin_poses_filename)

        bin_poses_tf = []

        for arm in bin_poses:
            for pose_name in bin_poses[arm]:
                bin_pose_id = pose_name.strip('_pose') + '_' + arm
                bin = pose_name[0:5]

                pose = bin_poses[arm][pose_name]
                orientation = pose['orientation']
                position = pose['position']

                rotation= [orientation['x'], orientation['y'], orientation['z'], orientation['w']]
                translation= [position['x'], position['y'], position['z']]

                bin_poses_tf.append({})
                bin_poses_tf[-1]['bin_pose_id'] = bin_pose_id
                bin_poses_tf[-1]['bin'] = bin
                bin_poses_tf[-1]['translation'] = translation
                bin_poses_tf[-1]['rotation'] = rotation


        return bin_poses_tf



    def frameExists(self, frame_id):
        '''

        @param frame_id:
        @return:
        '''
        with self._lock:

            if self._tfs.has_key(frame_id):
                return True

            else:
                return self._tf_listener.frameExists(frame_id)


    def _get_gripper_frame(self, frame):
        '''
        If the frame is (left/right)_gripper
        then get the transform using the baxter_interface
        @param frame:
        @return:
        '''

        arm = frame.strip('gripper') + 'arm'

        if arm == 'left_arm':
            limb = self._left_arm

        else:
            limb = self._right_arm

        endpoint_pose = limb.endpoint_pose()
        orientation = endpoint_pose['orientation']
        position = endpoint_pose['position']

        f_base_gripper = kdl.Frame(kdl.Rotation.Quaternion(orientation.x, orientation.y, orientation.z, orientation.w),
                                   kdl.Vector(position.x, position.y, position.z))

        return f_base_gripper


    def lookupTransform(self, frame_id, child_frame_id, time):
        '''

        @param frame_id:
        @param child_frame_id:
        @param time:
        @return:
        '''
        with self._lock:


            f_base_child = self._tfs.get(child_frame_id, None)
            f_base_parent = self._tfs.get(frame_id, None)

            gripper = child_frame_id.strip('_base')
            if child_frame_id == 'left_gripper' or child_frame_id == 'right_gripper':
                f_base_gripper = self._get_gripper_frame(gripper)
                f_base_child = copy.deepcopy(f_base_gripper)

            elif child_frame_id == 'left_gripper_base' or child_frame_id == 'right_gripper_base':

                f_base_gripper = self._get_gripper_frame(gripper)
                f_gripper_base_gripper = self._tfs.get(gripper)
                f_base_child = copy.deepcopy(f_base_gripper * f_gripper_base_gripper.Inverse())

            gripper = frame_id.strip('_base')
            if frame_id == 'left_gripper' or frame_id == 'right_gripper':
                f_base_gripper = self._get_gripper_frame(gripper)
                f_base_parent = copy.deepcopy(f_base_gripper)

            elif frame_id == 'left_gripper_base' or frame_id == 'right_gripper_base':
                f_base_gripper = self._get_gripper_frame(gripper)
                f_gripper_base_gripper = self._tfs.get(gripper)
                f_base_parent = copy.deepcopy(f_base_gripper * f_gripper_base_gripper.Inverse())


            if f_base_child == None or f_base_parent == None:
                try:
                    position, orientation = self._tf_listener.lookupTransform(frame_id,
                                                                              child_frame_id,
                                                                              rospy.Time(0))

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    raise tf.LookupException

                return (position, orientation)



            return posemath.toTf(f_base_parent.Inverse()*f_base_child)


def test_tf_cache():

    tf_cache = TFCache()
    import IPython
    IPython.embed()



if __name__ == '__main__':

    rospy.init_node('test_tf_cache')

    thread.start_new_thread(test_tf_cache, ())

    rospy.spin()