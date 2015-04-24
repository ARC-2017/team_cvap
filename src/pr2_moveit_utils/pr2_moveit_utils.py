#
#   pr2_moveit_utils.py
#
#   Created on: April 8, 2015
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

#


from tf_conversions import posemath
import PyKDL as kdl
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped


def transform_wrist_frame(T_tool, ft):
    '''
    :param T_tool: desired  *_gripper_tool_frame pose w.r.t. some reference frame. Can be of type kdl.Frame or geometry_msgs.Pose.
    :param ft: bool. True if the arm has a force-torque sensor
    :return: T_torso_wrist. geometry_msgs.Pose type. Wrist pose w.r.t. same ref frame as T_tool
    '''

    if ft:
        T_wrist_tool = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0.216, 0.0, 0.0))

    else:
        T_wrist_tool = kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0.180, 0.0, 0.0))

    if type(T_tool) is Pose:

        T_tool_ = posemath.fromMsg(T_tool)

    elif type(T_tool) is tuple and len(T_tool) is 2:
        T_tool_ = posemath.fromTf(T_tool)

    else:
        T_tool_ = T_tool


    T_wrist_ = T_tool_*T_wrist_tool.Inverse()

    T_wrist = posemath.toMsg(T_wrist_)

    return T_wrist

def plan_tool_frame(group, T_tool, base_frame_id='torso_lift_link', ft=False):
    '''
    Calculates a moveit motion plan of the PR2 arm's tool frame (l/r_gripper_tool_frame) w.r.t. some base ref frame (default: 'torso_lift_link')
    :param group: MoveGroupCommander type (left_arm or right_arm).
    :param T_tool: kdl.Frame or geometry_msgs.Pose. Desired pose of the tool frame w.r.t some base frame.
    :param base_frame_id: string, frame ID of the base in which T_tool is expressed. Default: 'torso_lift_link'
    :param ft: bool. True if the arm has a force-torque sensor, false otherwise
    :return: moveit plan from group.plan()
    '''

    T_wrist = transform_wrist_frame(T_tool, ft)

    T_wrist_stamped = PoseStamped()
    T_wrist_stamped.pose = T_wrist
    T_wrist_stamped.header.frame_id = base_frame_id
    T_wrist_stamped.header.stamp = rospy.Time.now()

    return group.plan(T_wrist_stamped)


def go_tool_frame(group, T_tool, base_frame_id='torso_lift_link', ft=False, wait=True):
    '''
    Plans and executes a moveit motion plan of the PR2 arm's tool frame (l/r_gripper_tool_frame) w.r.t. some base ref frame (default: 'torso_lift_link')
    :param group: MoveGroupCommander type (left_arm or right_arm).
    :param T_tool:  kdl.Frame or geometry_msgs.Pose. Desired pose of the tool frame w.r.t some base frame.
    :param base_frame_id: string, frame ID of the base in which T_tool is expressed. Default: 'torso_lift_link'
    :param ft: bool. True if the arm has a force-torque sensor, false otherwise
    :param wait: True/False to wait for execution of motion plan
    :return:
    '''


    T_wrist = transform_wrist_frame(T_tool, ft)

    T_wrist_stamped = PoseStamped()
    T_wrist_stamped.pose = T_wrist
    T_wrist_stamped.header.frame_id = base_frame_id
    T_wrist_stamped.header.stamp = rospy.Time.now()


    result = group.go(T_wrist_stamped, wait)
    
    if not result:
        raise Exception('NO MOTION PLAN FOUND')


    return result


