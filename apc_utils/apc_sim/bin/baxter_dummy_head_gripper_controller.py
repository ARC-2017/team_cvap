#!/usr/bin/env python

#   baxter_dummy_head_gripper_controller
#
#   Created on: April 27, 2016
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


import rospy

from std_msgs.msg import (
    Bool
)

import baxter_dataflow
import json

from baxter_core_msgs.msg import (
   HeadPanCommand,
   HeadState,
)

from baxter_core_msgs.msg import (
    EndEffectorCommand,
    EndEffectorProperties,
    EndEffectorState,
    EndpointState,
)

from sensor_msgs.msg import JointState
import threading

class BaxterDummyHeadGripperController:
    '''
    Class for simulating Baxter head and gripper in gazebo/rviz
    '''

    def __init__(self):
        '''

        '''

        # head
        self._head_cmd_sub = rospy.Subscriber('/robot/head/command_head_pan', HeadPanCommand,
                                                  self.head_pan_cmd_cb)

        self._head_state_pub = rospy.Publisher('/robot/head/head_state', HeadState, queue_size=1)
        self._head_state = HeadState(pan=0.0, isTurning=False, isNodding=False, isPanEnabled=True)


        # gripper
        self._left_gripper_cmd_sub = rospy.Subscriber('/robot/end_effector/left_gripper/command',
                                                      EndEffectorCommand, self.left_gripper_cmd_cb)

        self._right_gripper_cmd_sub = rospy.Subscriber('/robot/end_effector/right_gripper/command',
                                                      EndEffectorCommand, self.right_gripper_cmd_cb)

        # gripper states
        self._left_gripper_state = EndEffectorState()
        self._right_gripper_state = EndEffectorState()
        self.init_gripper_state(self._left_gripper_state)
        self.init_gripper_state(self._right_gripper_state)

        self._left_gripper_state_pub = rospy.Publisher('/robot/end_effector/left_gripper/state',
                                                       EndEffectorState, queue_size=1)
        self._right_gripper_state_pub = rospy.Publisher('/robot/end_effector/right_gripper/state',
                                                        EndEffectorState, queue_size=1)

        # gripper properties
        self._left_gripper_prop = EndEffectorProperties()
        self._right_gripper_prop = EndEffectorProperties()
        self.init_gripper_prop(self._left_gripper_prop, 'left')
        self.init_gripper_prop(self._right_gripper_prop, 'right')

        self._left_gripper_prop_pub = rospy.Publisher('/robot/end_effector/left_gripper/properties',
                                                      EndEffectorProperties, queue_size=1)
        self._right_gripper_prop_pub = rospy.Publisher('/robot/end_effector/right_gripper/properties',
                                                       EndEffectorProperties, queue_size=1)

        self._left_limb_endpoint_state = EndpointState()
        self._right_limb_endpoint_state = EndpointState()

        self.init_limb_endpoint_state(self._left_limb_endpoint_state, 'left')
        self.init_limb_endpoint_state(self._right_limb_endpoint_state, 'right')

        self._left_limb_endpoint_state_pub = rospy.Publisher('/robot/limb/left/endpoint_state', EndpointState, queue_size=1)
        self._right_limb_endpoint_state_pub = rospy.Publisher('/robot/limb/right/endpoint_state', EndpointState, queue_size=1)

        # joint state publisher
        self._js_pub = rospy.Publisher('/robot/joint_states', JointState, queue_size=1)
        self._joint_states = JointState()
        self.init_joint_state(self._joint_states)

        # timer for regularly publishing end effector state & properties, head state,
        # joint states
        self._timer = rospy.Timer(rospy.Duration(0.01), self.timer_cb)


    # head

    def head_pan_cmd_cb(self, msg):
        self._head_state.pan = msg.target
        self._joint_states.position =[msg.target]

    def init_gripper_state(self, gripper_state):
        gripper_state.enabled = True
        gripper_state.calibrated = True
        gripper_state.ready = True
        gripper_state.moving = False
        gripper_state.gripping = False
        gripper_state.missed = False
        gripper_state.error = False
        gripper_state.reverse = False
        gripper_state.state = json.JSONEncoder().encode({'vacuum': True, 'sucking': False, 'blowing': True})

    def init_gripper_prop(self, gripper_prop, arm):
        if arm=='left':
            gripper_prop.id = 0
        else:
            gripper_prop.id = 1

        gripper_prop.ui_type = EndEffectorProperties.SUCTION_CUP_GRIPPER

    def init_joint_state(self, joint_state):
        joint_state.name = ['head_pan']
        joint_state.position = [0.0]

    def init_limb_endpoint_state(self, endpoint_state, arm):
        endpoint_state.header.stamp = rospy.Time.now()
        endpoint_state.header.frame_id = arm + '_gripper'



    def left_gripper_cmd_cb(self, msg):
        return

    def right_gripper_cmd_cb(self, msg):
        return


    def gripper_cmd(self, gripper_state, msg):

        if msg.command == EndEffectorCommand.CMD_GO:
            return

        elif msg.command == EndEffectorCommand.CMD_STOP:
            return


    def timer_cb(self, event):
        '''
        regularly publish end effector state & properties, head state, joint states
        @param event:
        @return:
        '''
        if not rospy.is_shutdown():
            # head
            self._head_state_pub.publish(self._head_state)

            # grippers
            self._left_gripper_state_pub.publish(self._left_gripper_state)
            self._right_gripper_state_pub.publish(self._right_gripper_state)

            self._left_gripper_prop_pub.publish(self._left_gripper_prop)
            self._right_gripper_prop_pub.publish(self._right_gripper_prop)

            self._left_limb_endpoint_state_pub.publish(self._left_limb_endpoint_state)
            self._right_limb_endpoint_state_pub.publish(self._right_limb_endpoint_state)

            # joint states
            self._joint_states.header.stamp = rospy.Time.now()
            self._js_pub.publish(self._joint_states)



if __name__ == '__main__':
    rospy.init_node('baxter_dummy_head_gripper_controller')
    dummy_controller = BaxterDummyHeadGripperController()

    rospy.spin()



