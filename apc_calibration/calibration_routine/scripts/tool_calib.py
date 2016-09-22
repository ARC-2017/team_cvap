#!/usr/bin/env python

#   tool_calib
#
#   Created on: May 12, 2016
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
import argparse
from baxter_interface import DigitalIO

class ToolCalibration:
    def __init__(self, gripper, tool_name):

        self._tf_listener = tf.TransformListener()
        self._gripper = gripper
        self._tool_name = tool_name
        self._robot_base_frame = 'base'
        self._gripper_base_frame = gripper + '_gripper_base'
        self._pos_measurements = []
        self._orientation_measurement = kdl.Frame()

        # calibrations

        self._F_base_tool = kdl.Frame()
        self._F_wrist_tool = kdl.Frame()


        # IO buttons on the wrists

        self._get_measurement_button = DigitalIO(self._gripper + '_lower_button')
        self._stop_measurement_button = DigitalIO(self._gripper + '_upper_button')
        self._poll_rate = 100


    def get_transform(self):
        '''
        Gets transform between robot base frame and gripper base frame
        @return:
        '''

        while not rospy.is_shutdown():
            try:
                (trans, rot) = self._tf_listener.lookupTransform(self._robot_base_frame, self._gripper_base_frame, rospy.Time(0))
                break
            except:
                rospy.logerr(rospy.get_name() + ': could not get transform between ' +
                             self._robot_base_frame + ' and  ' + self._gripper_base_frame)
                rospy.sleep(1.0)

        return posemath.fromTf((trans, rot))

    def collect_pos_measurements(self):

        rospy.loginfo('====  ' + self._gripper + ' gripper Tool tip position calibration ====')
        rospy.loginfo('For each measurement, place the suction cup touching a fixed object in the world'
                      'from different orientations')
        rospy.loginfo('To collect a MEASUREMENT (MINIMUM 4!!!), press the GRAY CIRCULAR BUTTON on the wrist.')
        rospy.loginfo('To STOP collecting measurements, press the WHITE OVAL button on the wrist.')

        # collect at least 4 measurements
        while tool_calib.num_pos_measurements() < 4:
            while not self._get_measurement_button.state:
                rospy.sleep(1.0/self._poll_rate)
            self._pos_measurements.append(self.get_transform())
            rospy.loginfo('Got measurement. Total measurements: ' + str(self.num_pos_measurements()))
            rospy.sleep(5.0)


        # wait until one of the two buttons gets pressed
        while not self._get_measurement_button.state and not self._stop_measurement_button.state:
            rospy.sleep(1.0/self._poll_rate)

        # keep collecting measurements until the stop button is pressed
        while not self._stop_measurement_button.state:
            while not self._get_measurement_button.state:
                rospy.sleep(1.0/self._poll_rate)
                if self._stop_measurement_button.state:
                    return

            self._pos_measurements.append(self.get_transform())
            rospy.loginfo('Got measurement. Total measurements: ' + str(self.num_pos_measurements()))
            rospy.sleep(5.0)


    def calib_position(self):
        '''
        Calibrate the suction cup tip location. Pose it as a linear algebra problem
        @return:
        '''

        A = np.zeros((self.num_pos_measurements()*3, 6))
        b = np.zeros((self.num_pos_measurements()*3, 1))

        n = 0

        for measurement in self._pos_measurements:
            T = posemath.toMatrix(measurement)
            R = T[0:3, 0:3]
            p = T[0:3, 3]

            A[3*n:3*n+3, 0:3] = R
            A[3*n:3*n+3, 3:6] = -np.identity(3)

            b[3*n:3*n+3, 0] = -p

            n += 1

        self._A = A
        self._b = b

        sol = np.linalg.lstsq(A, b)
        x = sol[0]

        # pose of tool in wrist frame
        self._F_wrist_tool.p = kdl.Vector(x[0], x[1], x[2])

        # position of tool in robot base frame
        self._F_base_tool.p = kdl.Vector(x[3], x[4], x[5])

        return sol

    def collect_orientation_measurement(self):

        rospy.loginfo('====  ' + self._gripper + ' gripper Tool tip orientation calibration ====')
        rospy.loginfo('Place the gripper flat on a surface flat along the z-axis of the robot base frame '
                      'and then click on GRAY CIRCULAR BUTTON on the wrist to collect the measurement')

        while not self._get_measurement_button.state:
            rospy.sleep(1.0/self._poll_rate)
        self._orientation_measurement = self.get_transform()
        rospy.loginfo('Got orientation measurement.' )



    def calib_orientation(self):
        '''
        Determine orientation from placing the suction cup flat along the z-axis of the robot base frame
        @return:
        '''

        rospy.loginfo('==== Tool tip orientation calibration ====')
        rospy.loginfo('Place the suction cup facing down, flat on a surface which should have the same orientation'
                      'as the Baxter\'s base frame')

        # simply invert orientation of wrist w.r.t robot base frame
        # and flip the z axis to face down
        self._F_wrist_tool.M = (self._orientation_measurement.M.Inverse())*kdl.Rotation.RotX(np.pi)

        return



    def print_calib(self):

        rospy.loginfo('========= Tool Calibration =========')
        rospy.loginfo('Calibration of tool ' + self._tool_name + ' on the ' + self._gripper + ' gripper:')
        rospy.loginfo('[x, y, z, r, p ,y]: ')
        rospy.loginfo(str(self._F_wrist_tool.p) + ' | ' +  str(self._F_wrist_tool.M.GetRPY()))
        rospy.loginfo('[x, y, z, quaternion]: ')
        rospy.loginfo(str(self._F_wrist_tool.p) + ' | ' +  str(self._F_wrist_tool.M.GetQuaternion()))


    def get_filepath(self):
        '''
        gets filepath to calibration file
        @return: file path (str)
        '''

        rospack = rospkg.RosPack()
        path = rospack.get_path('calibration_data')
        path = path+'/extrinsics'

        return path

    def save_calib(self):
        '''
        Saves calibration to extrinsic calib file in data package
        @return:
        '''

        path = self.get_filepath()
        file = path + '/' + self._tool_name + '.txt'

        f = open(file, 'w')

        pose = ''

        for index in xrange(3):
            pose += str(self._F_wrist_tool.p[index]) + ' '

        q = self._F_wrist_tool.M.GetQuaternion()
        for index in xrange(4):
            pose += str(q[index]) + ' '

        f.write(pose)

        f.close()

        return

    def num_pos_measurements(self):
        '''
        Gives the number of collected measurements
        @return:
        '''

        return len(self._pos_measurements)

    def _test(self):

        p_w_t  = kdl.Vector(0.0, 0.1, 0.5)

        F_w_t = kdl.Frame(kdl.Rotation.Identity(), p_w_t)
        F_b_t = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(1.0, 0.4, -0.3))
        p_b_t = copy.deepcopy(F_b_t.p)

        r = 2*np.pi*np.random.rand(2, 1)-np.pi
        p = 2*np.pi*np.random.rand(2, 1)-np.pi
        y = 2*np.pi*np.random.rand(2, 1)-np.pi

        for roll in r:
            for pitch in p:
                for yaw in y:
                    p_noise = 0.004*np.random.randn(3, 1)
                    F_b_t.p = p_b_t + kdl.Vector(p_noise[0], p_noise[1], p_noise[2])
                    F_b_t.M = kdl.Rotation.RPY(roll, pitch, yaw)
                    F_b_w = F_b_t * F_w_t.Inverse()
                    self._pos_measurements.append(F_b_w)

        sol = self.calib()

        print(sol)

        return sol



if __name__ == "__main__":


    rospy.init_node('tool_calibration')

    parser = argparse.ArgumentParser(description='Calibration of suction cups (tool tip) on Baxter. Calibration is '
                                                 'printed out and saved in calibration_data/extrinsics/<tool_name>.txt. '
                                                 'The calibration is expressed w.r.t the gripper base '
                                                 '(e.g. left_gripper_base)')
    parser.add_argument('gripper', metavar='gripper', type=str, nargs='+',
                        help='select the gripper to calibrate (left or right)')

    parser.add_argument('tool_name', metavar="tool_name", type=str, nargs='+', default='tool',
                        help='name of the tool / suction cup. Specifies filename under which calibration will be saved')

    args = parser.parse_args()
    gripper = args.gripper[0]
    tool_name = args.tool_name[0]

    if not gripper=='left' and not gripper=='right':
        rospy.logerr('Wrong gripper argument for tool calibration, must be left or right')
        sys.exit()

    tool_calib = ToolCalibration(gripper, tool_name)
    #sol = tool_calib._test()

    rospy.sleep(1.0)

    rospy.loginfo('Tool calibration for Baxter!')

    # calibrate position of tool tip
    tool_calib.collect_pos_measurements()
    tool_calib.calib_position()

    # calibrate orientation of the tool tip
    tool_calib.collect_orientation_measurement()
    tool_calib.calib_orientation()

    tool_calib.print_calib()
    tool_calib.save_calib()
