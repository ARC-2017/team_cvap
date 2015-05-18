#!/usr/bin/env python

#   motion_utils
#
#   Created on: April 13, 2015
#   Authors:   Francisco Vina
#             fevb <at> kth.se
#

#  Copyright (c) 2015, Francisco Vina, CVAP, KTH
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
import moveit_commander
import numpy as np
import amazon_challenge_bt_actions.msg
import actionlib
from std_msgs.msg import String
from calibrateBase import baseMove
from pr2_controllers_msgs.msg import Pr2GripperCommand
import copy
import random
from moveit_commander import PlanningSceneInterface
import tf
from simtrack_nodes.srv import SwitchObjects

class BTMotion:

    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = amazon_challenge_bt_actions.msg.BTFeedback()
        self._result   = amazon_challenge_bt_actions.msg.BTResult()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
        self.pub_posed = rospy.Publisher('arm_posed', String, queue_size=10)
        self.pub_rate = rospy.Rate(30)

        self._planning_scene = PlanningSceneInterface()


        # get ROS parameters
        rospy.loginfo('Getting parameters...')
        while not rospy.is_shutdown():
            try:
                self._base_move_params = rospy.get_param('/base_move')
                self._timeout = rospy.get_param(name + '/timeout')
                self._sim = rospy.get_param(name + '/sim')
                self._base_pos_dict = rospy.get_param('/base_pos_dict')
                self._left_arm_joint_pos_dict = rospy.get_param('/left_arm_joint_pos_dict')
                self._right_arm_joint_pos_dict = rospy.get_param('/right_arm_joint_pos_dict')
                break
            except:
                rospy.sleep(random.uniform(0,2))
                continue

        self._exit = False

        while not rospy.is_shutdown():
            try:
                self._robot = moveit_commander.RobotCommander()
                self._left_arm = self._robot.get_group('left_arm')
                self._right_arm = self._robot.get_group('right_arm')
                self._arms = self._robot.get_group('arms')
                self._torso = self._robot.get_group('torso')
                self._head = self._robot.get_group('head')
                self._arms_dict = {'left_arm': self._left_arm, 'right_arm': self._right_arm}
                break
            except:
                rospy.sleep(random.uniform(0,2))
                continue



        self._bm = baseMove.baseMove(verbose=False)
        self._bm.setPosTolerance(self._base_move_params['pos_tolerance'])
        self._bm.setAngTolerance(self._base_move_params['ang_tolerance'])
        self._bm.setLinearGain(self._base_move_params['linear_gain'])
        self._bm.setAngularGain(self._base_move_params['angular_gain'])

        self._tf_listener = tf.TransformListener()

        self._next_task_sub = rospy.Subscriber("/amazon_next_task", String, self.get_task)

        self._l_gripper_pub = rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand)
        while not rospy.is_shutdown():
            try:
                self._tool_size = rospy.get_param('/tool_size', [0.16, 0.02, 0.04])
                self._contest = rospy.get_param('/contest', True)
                break
            except:
                rospy.sleep(random.uniform(0,1))
                continue

        if self._contest:
            self._length_tool = 0.18 + self._tool_size[0]
        else:
            self._length_tool = 0.216 + self._tool_size[0]

        self._as.start()
        rospy.loginfo('['+rospy.get_name()+']: ready!')



    def timer_callback(self, event):
        self._timer_started = True
        rospy.logerr('[' + rospy.get_name() + ']: TIMED OUT!')
        self._planning_scene.remove_attached_object('l_wrist_roll_link', 'grasped_object')
        self._planning_scene.remove_world_object('grasped_object')


        # pull the base back 60 cm

        self._left_arm.stop()
        self._right_arm.stop()

        base_pos_goal = [-1.42, self._bm.trans[1], self._bm.trans[2], 0.0, 0.0, 0.0]

        self._bm.goAngle(base_pos_goal[5])
        self._bm.goPosition(base_pos_goal[0:2])
        self._bm.goAngle(base_pos_goal[5])

        left_arm_joint_pos_goal = self._left_arm_joint_pos_dict['start']
        right_arm_joint_pos_goal = self._right_arm_joint_pos_dict['start']

        joint_pos_goal = left_arm_joint_pos_goal + right_arm_joint_pos_goal

        self._arms.set_joint_value_target(joint_pos_goal)
        self._arms.go()

        self._exit = True



    def execute_exit(self):
        if self._exit:
            self._success = False
            self.set_status('FAILURE')
            self._timer.shutdown()
            return True

        return False

    def execute_cb(self, goal):
        print 'bt motion execute callback'


    def shutdown_simtrack(self):
        # get simtrack switch objects service
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('/simtrack/switch_objects', 10.0)
                break
            except:
                rospy.loginfo('[' + rospy.get_name() + ']: waiting for simtrack switch object service')
                continue


        simtrack_switch_objects_srv = rospy.ServiceProxy('/simtrack/switch_objects', SwitchObjects)

        simtrack_switch_objects_srv.call()

    def init_as(self):

        self._planning_scene.remove_attached_object('l_wrist_roll_link', 'grasped_object')
        self._planning_scene.remove_world_object('grasped_object')
        self._timer_started = False
        self._exit=False
        self._timer = rospy.Timer(rospy.Duration(self._timeout), self.timer_callback, oneshot=True)

        self.shutdown_simtrack()

        rospy.sleep(2.0)


    def get_task(self, msg):
        text = msg.data
        text = text.replace('[','')
        text = text.replace(']','')
        words = text.split(',')
        self._bin = words[0]
        self._item = words[1]


    def get_row(self):
        '''
        For setting the torso height and arm pose
        '''

        while not rospy.is_shutdown():
            try:
                if self._bin=='bin_A' or self._bin=='bin_B' or self._bin=='bin_C':
                    return 'row_1'

                elif self._bin=='bin_D' or self._bin=='bin_E' or self._bin=='bin_F':
                    return 'row_2'

                elif self._bin=='bin_G' or self._bin=='bin_H' or self._bin=='bin_I':
                    return 'row_3'

                elif self._bin=='bin_J' or self._bin=='bin_K' or self._bin=='bin_L':
                    return 'row_4'

            except:
                pass

    def get_column(self):
        '''
        For setting the base pose
        '''
        while not rospy.is_shutdown():
            try:
                if self._bin=='bin_A' or self._bin=='bin_D' or self._bin=='bin_G' or self._bin=='bin_J':
                    return 'column_1'

                elif self._bin=='bin_B' or self._bin=='bin_E' or self._bin=='bin_H' or self._bin=='bin_K':
                    return 'column_2'

                elif self._bin=='bin_C' or self._bin=='bin_F' or self._bin=='bin_I' or self._bin=='bin_L':
                    return 'column_3'

            except:
                pass

    def go_joint_goal_async(self, group, joint_pos_goal, normalize_angles=False):

        q_goal = np.array(joint_pos_goal)
        if normalize_angles:
            q_goal = self.normalize_angles(joint_pos_goal)

        group.set_joint_value_target(joint_pos_goal)
        if not group.go(wait=False):
            return False

        q_now = np.array(group.get_current_joint_values())

        if normalize_angles:
            q_now = self.normalize_angles(q_now)


        q_tol = group.get_goal_joint_tolerance()
        if group.get_name()=='left_arm' or group.get_name()=='right_arm' or group.get_name()=='arms' or group.get_name()=='head':
            q_tol = 0.04

        elif group.get_name()=='torso':
            q_tol = 0.003

        t_print = rospy.Time.now()

        r = rospy.Rate(1.0)
        # check for preemption while the arm hasn't reach goal configuration
        while np.max(np.abs(q_goal-q_now)) > q_tol and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            q_now = np.array(group.get_current_joint_values())

            if normalize_angles:
                q_now = self.normalize_angles(q_now)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('action halted')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            if (rospy.Time.now()-t_print).to_sec()>3.0:
                t_print = rospy.Time.now()
                rospy.loginfo('[' + rospy.get_name() + ']: executing action')

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        return True


    def normalize_angles(self, q):
       '''
       normalize angles to -pi, pi
       '''
       q_normalized = np.mod(q, 2*np.pi)

       for i in xrange(np.size(q)):
           if q_normalized[i] > np.pi:
               q_normalized[i] = -(2*np.pi - q_normalized[i])

       return q_normalized



    def go_base_pos_async(self, base_pos_goal):

        angle = base_pos_goal[5]
        pos = base_pos_goal[0:2]
        r = rospy.Rate(20.0)

        # check for preemption while the base hasn't reach goal configuration
        while not self._bm.goAngle(angle, False) and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while moving base')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        while not self._bm.goPosition(pos, False) and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while moving base')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        while not self._bm.goAngle(angle, False) and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while moving base')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        return True

    def go_base_moveit_group_pos_async(self, base_pos_goal, group, joint_pos_goal, normalize_angles=False):
        angle = base_pos_goal[5]
        pos = base_pos_goal[0:2]
        r = rospy.Rate(20.0)

        q_goal = np.array(joint_pos_goal)
        if normalize_angles:
            q_goal = self.normalize_angles(joint_pos_goal)

        group.set_joint_value_target(joint_pos_goal)
        group.go(wait=False)

        q_now = np.array(group.get_current_joint_values())

        if normalize_angles:
            q_now = self.normalize_angles(q_now)


        q_tol = group.get_goal_joint_tolerance()
        if group.get_name()=='left_arm' or group.get_name()=='right_arm' or group.get_name()=='arms' or group.get_name()=='head':
            q_tol = 0.04

        elif group.get_name()=='torso':
            q_tol = 0.003

        t_print = rospy.Time.now()

        # check for preemption while the base hasn't reach goal configuration
        while not self._bm.goAngle(angle, False) and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while moving base')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        while not self._bm.goPosition(pos, False) and not rospy.is_shutdown():
            if self.execute_exit():
                return False

            q_now = np.array(group.get_current_joint_values())

            if normalize_angles:
                q_now = self.normalize_angles(q_now)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while moving base and ' + group.get_name())
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            if (rospy.Time.now()-t_print).to_sec()>3.0:
                t_print = rospy.Time.now()
                rospy.loginfo('[' + rospy.get_name() + ']: executing action')

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        while not self._bm.goAngle(angle, False) and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while moving base')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        # check for preemption while the arm hasn't reach goal configuration
        while np.max(np.abs(q_goal-q_now)) > q_tol and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            q_now = np.array(group.get_current_joint_values())

            if normalize_angles:
                q_now = self.normalize_angles(q_now)

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('action halted')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            if (rospy.Time.now()-t_print).to_sec()>3.0:
                t_print = rospy.Time.now()
                rospy.loginfo('[' + rospy.get_name() + ']: executing action')

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        if rospy.is_shutdown():
            return False

        return True

    def request_detection(self):
        client = actionlib.SimpleActionClient('amazon_detector', amazon_challenge_bt_actions.msg.DetectorAction)

        # Waits until the action server has started up and started
        # listening for goals.
        rospy.loginfo('Start Waiting')
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = amazon_challenge_bt_actions.msg.DetectorGoal(parameter=1)

            # Sends the goal to the action server.
        client.send_goal(goal)
        rospy.loginfo('Goal Sent')
            # Waits for the server to finish performing the action.
        client.wait_for_result()


    def set_status(self,status):
        if status == 'SUCCESS':
            self.pub_posed.publish("SUCCESS")
            rospy.sleep(1)
            self._feedback.status = 1
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

        elif status == 'FAILURE':
            self._feedback.status = 2
            self._result.status = self._feedback.status
            rospy.loginfo('Action %s: Failed' % self._action_name)
            self._as.set_succeeded(self._result)

        else:
            rospy.logerr('Action %s: has a wrong return status' % self._action_name)

    def open_l_gripper(self):
        gripper_command_msg = Pr2GripperCommand()
        gripper_command_msg.max_effort = 40.0
        gripper_command_msg.position = 10.0

        r = rospy.Rate(10.0)
        t_init = rospy.Time.now()

        while (rospy.Time.now()-t_init).to_sec()<3.0 and not rospy.is_shutdown():

            if self.execute_exit():
                return False

            self._l_gripper_pub.publish(gripper_command_msg)
              # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                rospy.loginfo('action halted while opening gripper')
                self._as.set_preempted()
                self._exit = True
                if self.execute_exit():
                    return False

            r.sleep()

        return True

    def move_l_arm_z(self, z_desired):
        '''
        computes straight line cartesian path in z direction
        :param z_desired:  of tool tip w.r.t. odom_combined
        :return:
        '''

        waypoints = []

        # waypoints.append(self._left_arm.get_current_pose().pose)

        wpose = copy.deepcopy(self._left_arm.get_current_pose().pose)
        wpose.position.z = z_desired + self._length_tool

        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self._left_arm.compute_cartesian_path(waypoints, 0.05, 0.0)


        # TODO make this asynchronous

        self._left_arm.execute(plan)

        return True

