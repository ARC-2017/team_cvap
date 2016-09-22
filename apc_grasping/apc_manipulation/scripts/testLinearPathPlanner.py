#!/usr/bin/env python

import IPython
from std_msgs.msg import Header
import rospy
from apc_linear_path_planner.srv import PlanLinearPath
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('LinearPathPlannerTest')

    rospy.wait_for_service('/apc/plan_linear_path')
    planService = rospy.ServiceProxy('/apc/plan_linear_path', PlanLinearPath)
    header = Header(stamp=rospy.Time.now())
    startState = JointState(header=header, name=['left_e0', 'left_e1', 'left_s0', 'left_s1',
                            'left_w0', 'left_w1', 'left_w2'], position=[-2.2361604935399617, 1.874908017992947,
                            0.2116893487281871, -0.24735440204652295, -0.8015049616701286, -1.1336118022473207,
                            -1.0227816903225997])
    header = Header(stamp=rospy.Time.now())
    endState = JointState(header=header, name=['left_e0', 'left_e1', 'left_s0', 'left_s1',
                          'left_w0', 'left_w1', 'left_w2'], position=[-1.559291470885523, 1.6321555583100802,
                          -0.3466796580621035, -1.0082088728376881, -1.015495281580144, -1.1807817114747972,
                          0.01112136071216925])

    result = planService(move_group='left_arm', start=startState, goal=endState)
    IPython.embed()
