#!/usr/bin/python

#   apc_plugin
#
#   Created on: April 21, 2016
#   Authors:   Sergio Caccamo        Francisco Vina
#              caccamo@kth.se          fevb@kth.se
#

#  Copyright (c) 2016, Sergio Caccamo, Francisco Vina, CVAP, KTH
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


import os
import rospy
import rospkg
import roslaunch

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QMainWindow, QGraphicsView, QIcon, QLineEdit, QTextEdit
from PyQt4.Qt import QPushButton, QLabel, QPixmap

import numpy as np
import copy
import random

from std_msgs.msg import String
from std_msgs.msg import Float32

from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf
from tf_conversions import posemath
import PyKDL as kdl

from apc_manipulation.msg import *
import actionlib

from apc_bt_comms.srv import SetBool, SetBoolRequest, SetBoolResponse

import baxter_interface
from baxter_core_msgs.msg import *

import json

class APCPlugin(Plugin):

#Components   @TODO: consider list and dynamic widget
    folder_name_launchfile = 'launch'
    launchfile_name_general = rospy.get_param('BUTTON_GEN_LAUNCH','openni2.launch')
    pkg_name_general = rospy.get_param('BUTTON_GEN_PKG', 'openni2_launch' )
    launchfile_name_bt = rospy.get_param('BUTTON_BT_LAUNCH','xs_bayer_rggb.launch')
    pkg_name_bt = rospy.get_param('BUTTON_BT_PKG', 'ueye_cam' )
    launchfile_name_perception= rospy.get_param('BUTTON_PERCEPTION_LAUNCH','openni2.launch')
    pkg_name_perception = rospy.get_param('BUTTON_PERCEPTION_PKG', 'openni2_launch' )
    launchfile_name_grasping = rospy.get_param('BUTTON_GRASPING_LAUNCH','openni2.launch')
    pkg_name_grasping = rospy.get_param('BUTTON_GRASPING_PKG', 'openni2_launch' )
    launchfile_name_calibration_1 = rospy.get_param('BUTTON_CALIBRATION_1_LAUNCH','openni2.launch')
    pkg_name_calibration_1 = rospy.get_param('BUTTON_CALIBRATION_1_PKG', 'openni2_launch' )
    launchfile_name_calibration_2 = rospy.get_param('BUTTON_CALIBRATION_2_LAUNCH','openni2.launch')
    pkg_name_calibration_2 = rospy.get_param('BUTTON_CALIBRATION_2_PKG', 'openni2_launch' )
    launchfile_name_calibration_3 = rospy.get_param('BUTTON_CALIBRATION_3_LAUNCH','openni2.launch')
    pkg_name_calibration_3 = rospy.get_param('BUTTON_CALIBRATION_3_PKG', 'openni2_launch' )


    args_general =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_general, launchfile_name_general])
    args_bt =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_bt, launchfile_name_bt])
    args_perception =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_perception, launchfile_name_perception])
    args_grasping =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_grasping, launchfile_name_grasping])
    args_calibration_1 =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_calibration_1, launchfile_name_calibration_1])
    args_calibration_2 =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_calibration_2, launchfile_name_calibration_2])
    args_calibtration_3 =  roslaunch.rlutil.resolve_launch_arguments([pkg_name_calibration_3, launchfile_name_calibration_3])

    runner_general = []
    runner_bt = []
    runner_perception = []
    runner_grasping = []
    runner_calibration_1 = []
    runner_calibration_2 = []
    runner_calibration_3 = []

    #icons
    _icon_node_start = QIcon.fromTheme('media-playback-start')
    _icon_node_stop = QIcon.fromTheme('media-playback-stop')
    _icon_respawn_toggle = QIcon.fromTheme('view-refresh')


    def __init__(self, context):
        super(APCPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('APCPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QMainWindow()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_apc'), 'resource', 'apc.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('APCPlugin')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)


        # Add function calls for each of the buttons.
        self._widget.button_component_general.clicked.connect(self.button_component_general_clicked)
        self._widget.button_component_0.clicked.connect(self.button_component_0_clicked)
        self._widget.button_component_1.clicked.connect(self.button_component_1_clicked)
        self._widget.button_component_2.clicked.connect(self.button_component_2_clicked)
        self._widget.button_calibration_0.clicked.connect(self.button_calibration_0_clicked)
        self._widget.button_calibration_1.clicked.connect(self.button_calibration_1_clicked)
        self._widget.button_calibration_2.clicked.connect(self.button_calibration_2_clicked)

        self._widget.toggle_arm_button.clicked[bool].connect(self.toggle_arm_button_clicked)

        self._widget.home_button.clicked[bool].connect(self.home_button_clicked)
        self._widget.bin_a_button.clicked[bool].connect(self.bin_a_button_clicked)
        self._widget.bin_b_button.clicked[bool].connect(self.bin_b_button_clicked)
        self._widget.bin_c_button.clicked[bool].connect(self.bin_c_button_clicked)
        self._widget.bin_d_button.clicked[bool].connect(self.bin_d_button_clicked)
        self._widget.bin_e_button.clicked[bool].connect(self.bin_e_button_clicked)
        self._widget.bin_f_button.clicked[bool].connect(self.bin_f_button_clicked)
        self._widget.bin_g_button.clicked[bool].connect(self.bin_g_button_clicked)
        self._widget.bin_h_button.clicked[bool].connect(self.bin_h_button_clicked)
        self._widget.bin_i_button.clicked[bool].connect(self.bin_i_button_clicked)
        self._widget.bin_j_button.clicked[bool].connect(self.bin_j_button_clicked)
        self._widget.bin_k_button.clicked[bool].connect(self.bin_k_button_clicked)
        self._widget.bin_l_button.clicked[bool].connect(self.bin_l_button_clicked)

        self._widget.move_to_bin_button.clicked[bool].connect(self.move_to_bin_button_clicked)

        self._widget.left_gripper_button.clicked[bool].connect(self.left_gripper_button_clicked)
        self._widget.right_gripper_button.clicked[bool].connect(self.right_gripper_button_clicked)
        self._widget.left_suction_off_button.clicked[bool].connect(self.left_suction_off_button_clicked)
        self._widget.right_suction_off_button.clicked[bool].connect(self.right_suction_off_button_clicked)

        self._widget.start_bt_button.clicked[bool].connect(self.start_bt_button_clicked)
        self._widget.pause_bt_button.clicked[bool].connect(self.pause_bt_button_clicked)

        self._widget.click_to_pick_button.clicked[bool].connect(self.click_to_pick_button_clicked)
        self._widget.place_button.clicked[bool].connect(self.place_button_clicked)

        #Set icons to buttons
        self._widget.button_component_general.setIcon(self._icon_node_start)
        self._widget.button_component_diagn.setIcon(self._icon_node_start)
        self._widget.button_component_0.setIcon(self._icon_node_start)
        self._widget.button_component_1.setIcon(self._icon_node_start)
        self._widget.button_component_2.setIcon(self._icon_node_start)
        self._widget.button_calibration_0.setIcon(self._icon_node_start)
        self._widget.button_calibration_1.setIcon(self._icon_node_start)
        self._widget.button_calibration_2.setIcon(self._icon_node_start)


        self._json_data = None


        # for storing the joint states
        self._left_joint_state = [0.0 for x in xrange(7)]
        self._right_joint_state = [0.0 for x in xrange(7)]

        # for selecting the arm
        self._arm = 'left_arm'

        # subscribers
        self._joint_state_sub = rospy.Subscriber('/robot/joint_states', JointState, self.joint_state_cb)
        self._tf_listener = tf.TransformListener()
        self._move_arm_action_server_name = '/apc/manipulation/move_arm'
        self._move_arm_client = actionlib.SimpleActionClient(self._move_arm_action_server_name, MoveArmAction)

        self._pick_object_action_server_name = '/apc/manipulation/pick_object'
        self._pick_object_client = actionlib.SimpleActionClient(self._pick_object_action_server_name, PickObjectAction)

        self._place_object_action_server_name = '/apc/manipulation/place_object'
        self._place_object_client = actionlib.SimpleActionClient(self._place_object_action_server_name, PlaceObjectAction)


        # baxter grippers
        self._grippers_initialized = False


        self._timer = rospy.Timer(rospy.Duration(1.0), self.timer_cb)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for rqt_apc plugin')
        # group.add_argument('bagfiles', type=argparse.FileType('r'), nargs='*', default=[], help='Bagfiles to load')

    def get_desperation(self):
        desperation = rospy.get_param('/apc/task_manager/desperation', 1)

        if desperation == 1:
            return "All is good, I'm so confident"
        elif desperation == 2:
            return "Ooops, using plan B"
        elif desperation == 3:
            return "Double oops, using plan C"
        elif desperation == 4:
            return "Triple oops, using plan D"

    def timer_cb(self, event):

        # get the demo parameter
        while not rospy.is_shutdown():
            try:
                demo = rospy.get_param('/apc/demo', 'NO DEMO PARAM')
                object = rospy.get_param("/apc/task_manager/target_object", 'NO OBJECT')
                bin = rospy.get_param("/apc/task_manager/target_bin", 'NO BIN')
                break
            except:
                rospy.sleep(random.uniform(0, 1))
                continue

        if rospy.is_shutdown():
            return

        if self._json_data == None:
            self.init_json()

        self._widget.demo_label.setText(demo)
        self._widget.action_label.setText(rospy.get_param('/apc/bt/action', 'no action'))
        self._widget.action_label_2.setText(self.get_desperation())
        self._widget.json_filename_label.setText('apc_bt_launcher/data/' + self._json_filename)

        self._widget.current_arm_label.setText(self._arm)

        self._widget.item_label.setText(object)
        self._widget.bin_label.setText(bin)

        # check if gripper state has been published and initialize the grippers
        if not self._grippers_initialized:
            try:
                rospy.wait_for_message('/robot/end_effector/left_gripper/state', EndEffectorState, 1.0)
                rospy.wait_for_message('/robot/end_effector/right_gripper/state', EndEffectorState, 1.0)
                self._left_gripper = baxter_interface.Gripper('left')
                self._right_gripper = baxter_interface.Gripper('right')
                self._grippers_initialized = True
            except rospy.ROSException:
                self._grippers_initialized = False
                rospy.logwarn('[' + rospy.get_name() + ']: waiting for gripper state on topic /robot/end_effector/*_gripper/state')


        # set the joint states

        left_joint_state_str = str(['%.2f' % q for q in self._left_joint_state])
        right_joint_state_str = str(['%.2f' % q for q in self._right_joint_state])

        if self._arm == "left_arm":
            self._widget.joint_pos_label.setText(str(left_joint_state_str))
            gripper_frame = 'left_gripper'
        else:
            self._widget.joint_pos_label.setText(str(right_joint_state_str))
            gripper_frame = 'right_gripper'

        # get the transform for the gripper tip

        try:
            gripper_pose = posemath.fromTf(self._tf_listener.lookupTransform('base', gripper_frame, rospy.Time(0)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        rpy = gripper_pose.M.GetRPY()
        pose = ['' for x in xrange(6)]

        for index in xrange(3):
            pose[index] = '%.2f' % gripper_pose.p[index]
            pose[index+3] = '%.2f' % rpy[index]

        self._widget.pose_label.setText(str(pose))

    def init_json(self):

        # get the demo parameter
        while not rospy.is_shutdown():
            try:
                self._json_filename = rospy.get_param('/apc/task_manager/json_filename', 'NO JSON FILENAME')
                break
            except:
                rospy.sleep(random.uniform(0, 1))
                continue

        # open the json file and get the object to pick according to the bin
        rospack = rospkg.RosPack()
        json_path = rospack.get_path('apc_bt_launcher') + '/data/'
        json_file = open(json_path + self._json_filename + '.json')

        self._json_data = json.load(json_file)

        json_file.close()



    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass


    # Callbacks ------------------
    def joint_state_cb(self, msg):
        '''
        Get the joint states
        :param msg: joint state message
        :return:
        '''

        joint_names = msg.name
        joint_index = 0
        for joint_name in joint_names:
            joint_number = self.get_joint_number(joint_name)
            if joint_number>=0:
                if joint_name[:4] == "left":
                    self._left_joint_state[joint_number] = msg.position[joint_index]
                elif joint_name[:5] == 'right':
                    self._right_joint_state[joint_number] = msg.position[joint_index]

            joint_index += 1


    def get_joint_number(self, joint_name):
        '''
        Converts Baxter joint names e.g. 'left_w0' to number index from 1...7
        :param joint_name: string with joint name
        :return: joint_index: int between 0 and 6, -1 if not valid joint
        '''

        # first strip the prefix 'left_' or 'right_'
        joint = ''
        joint = joint_name.strip('left_')
        joint = joint.strip('right_')

        joint_dict = {'s0': 0, 's1': 1, 'e0': 2, 'e1': 3, 'w0': 4, 'w1': 5, 'w2': 6}

        if joint_dict.has_key(joint):
            return joint_dict[joint]

        else:
            return -1


    # Buttons --------------------

    # ============= Arm stuff ========================

    def toggle_arm_button_clicked(self):
        '''
        switch arm 'left' <--> 'right'
        :return:
        '''

        if self._arm == 'left_arm':
            self._arm = 'right_arm'

        else:
            self._arm = 'left_arm'

    def left_gripper_button_clicked(self):
        '''
        Turn on/off the left gripper
        :return:
        '''
        self.activate_gripper('left_arm')

    def right_gripper_button_clicked(self):
        '''
        turn on/off the right gripper
        :return:
        '''
        self.activate_gripper('right_arm')

    def activate_gripper(self, arm):
        '''
        turn on a gripper
        :param arm:
        :return:
        '''
        # only run if grippers have been detected
        if not self._grippers_initialized:
            return False

        if arm=='left_arm':
            self._left_gripper.command_suction(timeout=15.0)

        else:
            self._right_gripper.command_suction(timeout=15.0)

        return True

    def left_suction_off_button_clicked(self):

        if not self._grippers_initialized:
            return

        self._left_gripper.stop()

    def right_suction_off_button_clicked(self):

        if not self._grippers_initialized:
            return

        self._right_gripper.stop()



    def home_button_clicked(self):
        self.move_arm_bin('HOME')

    def bin_a_button_clicked(self):
        self.move_arm_bin('bin_A')

    def bin_b_button_clicked(self):
        self.move_arm_bin('bin_B')

    def bin_c_button_clicked(self):
        self.move_arm_bin('bin_C')

    def bin_d_button_clicked(self):
        self.move_arm_bin('bin_D')

    def bin_e_button_clicked(self):
        self.move_arm_bin('bin_E')

    def bin_f_button_clicked(self):
        self.move_arm_bin('bin_F')

    def bin_g_button_clicked(self):
        self.move_arm_bin('bin_G')

    def bin_h_button_clicked(self):
        self.move_arm_bin('bin_H')

    def bin_i_button_clicked(self):
        self.move_arm_bin('bin_I')

    def bin_j_button_clicked(self):
        self.move_arm_bin('bin_J')

    def bin_k_button_clicked(self):
        self.move_arm_bin('bin_K')

    def bin_l_button_clicked(self):
        self.move_arm_bin('bin_L')

    def move_to_bin_button_clicked(self):
        self.move_arm_bin(self._widget.bin_text_input.toPlainText())

    def move_arm_bin(self, bin):
        self._bin = bin
        goal = MoveArmGoal(goalBin=bin, arm=self._arm)
        self._move_arm_client.send_goal(goal)

    def click_to_pick_button_clicked(self):

        # only do this if JSON file has been initialized
        if not self._json_data == None:

            # get the target object for the bin
            for bin in self._json_data['work_order']:
                if bin['bin'] == self._bin:
                    obj = bin['item']
                    break

            goal = PickObjectGoal(targetObject=obj, bin_id=self._bin, arm=self._arm)
            self._pick_object_client.send_goal(goal)

        else:
            rospy.logwarn('[' + rospy.get_name() + ']: json file not initialized.')

    def place_button_clicked(self):

        return 0



    # ============= BT stuff ========================

    def start_bt_button_clicked(self):
        '''

        :return:
        '''
        rospy.set_param('/apc/task_manager/running', True)


    def pause_bt_button_clicked(self):
        '''

        :param self:
        :return:
        '''
        rospy.set_param('/apc/task_manager/running', False)

    # ============= SYS stuff ========================

    def button_component_general_clicked(self):
        if self._widget.button_component_general.isChecked():
            self._widget.button_component_general.setIcon(self._icon_node_stop)
            #launchfile = os.path.join(rospkg.RosPack().get_path(pkg_name), folder_name_launchfile, launchfile_name)
            #print( launchfile)
            self.runner_general = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_general)
            self.runner_general.start()
            print "Running the KTH APC system"
        else:
            self.runner_general.shutdown()
            self._widget.button_component_general.setIcon(self._icon_node_start)


    def button_component_0_clicked(self):
        if self._widget.button_component_0.isChecked():
            self._widget.button_component_0.setIcon(self._icon_node_stop)

            print "Running the BT (standalone)"
            self.runner_bt = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_bt)
            self.runner_bt.start()
        else:
            self.runner_bt.shutdown()
            self._widget.button_component_0.setIcon(self._icon_node_start)
            print "Killing the BT (standalone)"

    def button_component_1_clicked(self):
        if self._widget.button_component_1.isChecked():
            self._widget.button_component_1.setIcon(self._icon_node_stop)

            print "Running the PERCEPTION component (standalone)"
            self.runner_perception = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_perception)
            self.runner_perception.start()
        else:
            self.runner_perception.shutdown()
            self._widget.button_component_1.setIcon(self._icon_node_start)
            print "Killing the PERCEPTION component (standalone)"

    def button_component_2_clicked(self):
        if self._widget.button_component_2.isChecked():
            self._widget.button_component_2.setIcon(self._icon_node_stop)

            print "Running the GRASPING component (standalone)"
            self.runner_grasping = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_grasping)
            self.runner_grasping.start()
        else:
            self.runner_grasping.shutdown()
            self._widget.button_component_2.setIcon(self._icon_node_start)
            print "Killing the GRASPING component (standalone)"

    def button_calibration_0_clicked(self):
        if self._widget.button_calibration_0.isChecked():
            self._widget.button_calibration_0.setIcon(self._icon_node_stop)

            print "Running the CALIBRATION for the Kinect chest"
            self.runner_calibration_1 = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_calibration_1)
            self.runner_calibration_1.start()
        else:
            self.runner_calibration_1.shutdown()
            self._widget.button_calibration_0.setIcon(self._icon_node_start)
            print "kILLING the CALIBRATION for the Kinect chest"

    def button_calibration_1_clicked(self):
        if self._widget.button_calibration_1.isChecked():
            self._widget.button_calibration_1.setIcon(self._icon_node_stop)

            print "Running the CALIBRATION for the Kinect head"
            self.runner_calibration_2 = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_calibration_2)
            self.runner_calibration_2.start()
        else:
            self.runner_calibration_2.shutdown()
            self._widget.button_calibration_1.setIcon(self._icon_node_start)
            print "kILLING the CALIBRATION for the Kinect head"

    def button_calibration_2_clicked(self):
        if self._widget.button_calibration_2.isChecked():
            self._widget.button_calibration_2.setIcon(self._icon_node_stop)

            print "Running the CALIBRATION for the IDS camera"
            self.runner_calibration_3 = roslaunch.parent.ROSLaunchParent( rospy.get_param("/run_id"), self.args_calibration_3)
            self.runner_calibration_3.start()
        else:
            self.runner_calibration_3.shutdown()
            self._widget.button_calibration_2.setIcon(self._icon_node_start)
            print "kILLING the CALIBRATION for the IDS camera"
