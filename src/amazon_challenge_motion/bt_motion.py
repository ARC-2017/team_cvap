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


class BTMotion():

    def __init__(self, name):
        # create messages that are used to publish feedback/result
        self._feedback = amazon_challenge_bt_actions.msg.BTFeedback()
        self._result   = amazon_challenge_bt_actions.msg.BTResult()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.BTAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.pub_posed = rospy.Publisher('arm_posed', String, queue_size=10)
        self.pub_rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            try:
                self._left_arm = moveit_commander.MoveGroupCommander('left_arm')
                self._right_arm = moveit_commander.MoveGroupCommander('right_arm')
                self._arms = moveit_commander.MoveGroupCommander('arms')
                self._torso = moveit_commander.MoveGroupCommander('torso')
                self._arms_dict = {'left_arm': self._left_arm, 'right_arm': self._right_arm}
                break
            except:
                pass

        self._bm = baseMove.baseMove(verbose=False)
        self._bm.setPosTolerance(0.02)
        self._bm.setAngTolerance(0.006)
        self._bm.setLinearGain(0.4)
        self._bm.setAngularGain(1)

        rospy.Subscriber("/amazon_next_task", String, self.get_task)


    def execute_cb(self, goal):
        rospy.loginfo('test execute cb')


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

    def go_joint_goal_async(self, group, joint_pos_goal):

        q_goal = self.normalize_angles(joint_pos_goal)
        group.set_joint_value_target(joint_pos_goal)
        group.go(wait=False)


        q_now = self.normalize_angles(group.get_current_joint_values())


        q_tol = group.get_goal_joint_tolerance()
        if group.get_name()=='left_arm' or group.get_name()=='right_arm' or group.get_name()=='arms':
            q_tol = 0.02

        elif group.get_name()=='torso':
            q_tol = 0.003

        t_print = rospy.Time.now()

        # TODO: add timeouts for motion
        # check for preemption while the arm hasn't reach goal configuration
        while np.max(np.abs(q_goal-q_now)) > q_tol and not rospy.is_shutdown():
            q_now = self.normalize_angles(group.get_current_joint_values())

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted')
                self._as.set_preempted()
                self._success = False
                return False

            if (rospy.Time.now()-t_print).to_sec()>3.0:
                t_print = rospy.Time.now()
                rospy.loginfo('[pregrasp_server]: executing action')

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            rospy.sleep(0.1)

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
        r = rospy.Rate(100.0)

        # check for preemption while the base hasn't reach goal configuration
        while not self._bm.goAngle(angle) and not rospy.is_shutdown():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted while moving base')
                self._as.set_preempted()
                self._success = False
                return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        while not self._bm.goPosition(pos) and not rospy.is_shutdown():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted while moving base')
                self._as.set_preempted()
                self._success = False
                return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

        while not self._bm.goAngle(angle) and not rospy.is_shutdown():

            # check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                #HERE THE CODE TO EXECUTE WHEN THE  BEHAVIOR TREE DOES HALT THE ACTION
                group.stop()
                rospy.loginfo('[pregrasp_server]: action halted while moving base')
                self._as.set_preempted()
                self._success = False
                return False

            #HERE THE CODE TO EXECUTE AS LONG AS THE BEHAVIOR TREE DOES NOT HALT THE ACTION
            r.sleep()

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