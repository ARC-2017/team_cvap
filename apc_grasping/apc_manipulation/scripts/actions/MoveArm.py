#!/usr/bin/env python
"""
    This class implementes a ROSAction server for moving the arm to some goal position.

    @author: Joshua Haustein (haustein@kth.se)
"""

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from utils.ArgumentsCollector import ArgumentsCollector
from apc_manipulation.msg import MoveArmAction, MoveArmActionResult
import utils.ArgumentsCollector
from utils.MoveItInterface import MoveResult
from utils.MoveItInterface import PreemptionException, MoveItProblemException
import traceback


class MoveArm(object):
    """ ROSAction server for moving the arm. """
    MoveArmFailed, MoveArmSuccess, MoveArmPreempted, MoveArmError = [-1, 1, 0, -2]

    def __init__(self, name, moveitInterface):
        self._server = actionlib.SimpleActionServer(name, MoveArmAction, self.execute_cb, False)
        self._server.register_preempt_callback(self.preemptCallback)
        self._moveitInterface = moveitInterface
        self._argumentsRetriever = ArgumentsCollector('/apc/manipulation/move_arm/input',
                                                      ['arm', 'goalBin'])
        self._result = MoveArmActionResult()
        self._server.start()

    def returnError(self, errorMsg, error=None):
        rospy.logerr(errorMsg)
        if error is not None:
            stackMsg = traceback.format_exc(error)
            rospy.logerr(stackMsg)
        self._result.result.success = False
        self._server.set_aborted(self._result.result)
        return MoveArm.MoveArmFailed

    def returnSuccess(self):
        rospy.loginfo('MoveArm executed successfully.')
        self._result.result.success = True
        self._server.set_succeeded(self._result.result)
        return MoveArm.MoveArmSuccess

    def returnFail(self):
        rospy.loginfo('MoveArm executed properly, but failed.')
        self._result.result.success = False
        self._server.set_aborted(self._result.result)
        return MoveArm.MoveArmFailed

    def preemptCallback(self):
        self._moveitInterface.setPreempted(True)

    def returnPreempted(self):
        rospy.loginfo('Preempting MoveArm action. These people today.... No patience!')
        self._result.result.success = False
        self._server.set_preempted()
        return MoveArm.MoveArmPreempted

    def moveToConfiguration(self, goal, arm):
        if not isinstance(goal, dict):
            errorMsg = 'Goal configuration of type %s is not supported yet.' % repr(type(goal))
            return None, errorMsg
        (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(
            config=goal, arm=arm)
        return moveArmResult, None

    def moveToPose(self, goal, arm, goalName):
        if not isinstance(goal, PoseStamped):
            errorMsg = 'Goal of type %s is not supported yet.' % repr(type(goal))
            return None, errorMsg
            # moveGroup.set_position_target(position)
            # moveGroup.set_joint_value_target(goal.configuration)
        # If we reach this point, we have a valid goal pose
        seedIk = self._moveitInterface.getIKSeed(arm, goalName)
        (moveArmResult, traj) = self._moveitInterface.moveToPoses(
            poses=[goal], arm=arm, seedIk=seedIk)
        return moveArmResult, None

    def moveArm(self, actionGoal):
        bMoveToPose = False
        bMoveToConfiguration = False

        # If we have arguments try to move the arm
        result = None
        try:
            # Get hold of MoveIt
            self._moveitInterface.acquireLock()
            self._moveitInterface.setPreempted(False)
        except ValueError as error:
            return (MoveArm.MoveArmError, 'Failed to get access to robot: ' + repr(error))

        try:
            # Set the goal
            goal = actionGoal.goalBin
            goalName = ""
            if isinstance(goal, dict):
                try:
                    goal = utils.ArgumentsCollector.mapToPose(goal)
                    bMoveToPose = True
                except KeyError as error:
                    return (MoveArm.MoveArmError, 'Could not parse goal ' + repr(goal))
            elif isinstance(goal, str):
                goalName = goal
                goal = self._moveitInterface.getNamedConfiguration(goalName, actionGoal.arm)
                bMoveToConfiguration = True
                if goal is None:
                    bMoveToConfiguration = False
                    goalNameArm = goalName + '_' + actionGoal.arm
                    if self._moveitInterface.isPoseKnown(goalName):
                        bMoveToPose = True
                        goal = self._moveitInterface.getTransform(goalName)
                    elif self._moveitInterface.isPoseKnown(goalNameArm):
                        bMoveToPose = True
                        goal = self._moveitInterface.getTransform(goalNameArm)
                    else:
                        return (MoveArm.MoveArmError, 'Do not know pose ' + goalName + '. Can not move there.')
            if bMoveToPose:
                result, msg = self.moveToPose(goal, actionGoal.arm, goalName)
            elif bMoveToConfiguration:
                result, msg = self.moveToConfiguration(goal, actionGoal.arm)
            else:
                rospy.logwarn('Do not have a goal. Not moving anywhere!')
            if result is None:
                return (MoveArm.MoveArmError, msg)
        except PreemptionException as preemptErr:
            rospy.logwarn('MoveArm action was preempted:' + repr(preemptErr))
            return (MoveArm.MoveArmPreempted, '')
        except MoveItProblemException as moveitErr:
            rospy.logerr('Use MoveIt they said.... it does not work. Something is wrong with MoveIt: ' + repr(moveitErr))
            return (MoveArm.MoveArmError, 'MoveItError')
        except Exception as error:
            stackMsg = traceback.format_exc(error)
            rospy.logerr(stackMsg)
            return (MoveArm.MoveArmError, 'Could not move arm because:\n' + error.message)
        finally:
            self._moveitInterface.releaseLock()

        if result == MoveResult.Success:
            return (MoveArm.MoveArmSuccess, '')
        else:
            return (MoveArm.MoveArmFailed, '')

    def returnResult(self, result, msg):
        if result == MoveArm.MoveArmSuccess:
            return self.returnSuccess()
        elif result == MoveArm.MoveArmFailed:
            return self.returnFail()
        elif result == MoveArm.MoveArmError:
            return self.returnError(msg)
        elif result == MoveArm.MoveArmPreempted:
            return self.returnPreempted()
        return self.returnError('Unknown result ' + repr(msg))

    def execute_cb(self, actionGoal):
        if actionGoal.arm == 'both_arms':
            firstArm = 'left_arm' if self._moveitInterface.getCurrentHeadSide() == 'right' else 'right_arm'
            secondArm = 'right_arm' if firstArm == 'left_arm' else 'left_arm'
            actionGoal.arm = firstArm
            (result, msg) = self.moveArm(actionGoal)
            if result == MoveArm.MoveArmSuccess:
                actionGoal.arm = secondArm
                (result, msg) = self.moveArm(actionGoal)
            return self.returnResult(result, msg)
        else:
            (result, msg) = self.moveArm(actionGoal)
            return self.returnResult(result, msg)
