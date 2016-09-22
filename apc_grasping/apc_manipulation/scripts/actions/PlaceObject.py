#!/usr/bin/env python
"""
    This class implementes a ROSAction server for placing an object.

    @author: Joshua Haustein (haustein@kth.se)
"""

import rospy
import actionlib
from utils.MoveItInterface import MoveResult
from apc_manipulation.msg import PlaceObjectAction, PlaceObjectActionResult
import utils.ArgumentsCollector
from utils.MoveItInterface import PreemptionException, MoveItProblemException
import PyKDL as kdl
import math
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
import utils.BinPoseMath
from tf_conversions import posemath
import traceback
import IPython


class PlaceObject(object):
    """ ROSAction server for placing an object. """

    def __init__(self, name, moveitInterface):
        self._server = actionlib.SimpleActionServer(name, PlaceObjectAction, self.execute_cb, False)
        self._server.register_preempt_callback(self.preemptCallback)
        self._moveitInterface = moveitInterface
        self._pubPoses = rospy.Publisher('place_poses', PoseArray, queue_size=1)
        self._parameterRetriever = utils.ArgumentsCollector.ArgumentsCollector('/apc/manipulation/place_object/params',
                                                      ['planning_time_out', 'pos_tolerance', 'rot_tolerance',
                                                       'safe_stowing_distance', 'stowing_distance', 'stowing_retreat_distance'
                                                       'stowing_information'])
        self._params = {}
        self._params['planning_time_out'] = 5
        self._params['pos_tolerance'] = 0.05
        self._params['rot_tolerance'] = 0.2
        self._params['safe_stowing_distance'] = 0.08
        self._params['stowing_distance'] = 0.15
        self._params['stowing_retreat_distance'] = 0.04
        self._params['lowering_distance'] = 0.05
        self._params['stowing_information'] = {'bin_types': {}, 'default_object': {}}
        for binName in ['bin_A', 'bin_B', 'bin_C', 'bin_J', 'bin_K', 'bin_L']:
            self._params['stowing_information']['bin_types'][binName] = 'large'
        for binName in ['bin_D', 'bin_E', 'bin_F', 'bin_G', 'bin_H', 'bin_I']:
            self._params['stowing_information']['bin_types'][binName] = 'small'
        self._params['stowing_information']['default_object'] = {'large': {'x': -0.3, 'y': 0.0, 'z': 0.19, 'rot': 0.0, 'rot_electric':0.0},
                                                                 'small': {'x': -0.3, 'y': 0.0, 'z': 0.16, 'rot': 0.0, 'rot_electric':0.0}}
        self._result = PlaceObjectActionResult()
        self._server.start()

    def updateParameters(self):
        params = self._parameterRetriever.getArguments(True)
        for key in params.keys():
            self._params[key] = params[key]

    def returnError(self, errorMsg, error=None):
        rospy.logerr(errorMsg)
        if error is not None:
            stackMsg = traceback.format_exc(error)
            rospy.logerr(stackMsg)
        self._result.result.success = False
        self._result.result.had_object = True
        self._server.set_aborted(self._result.result)
        return -1

    def returnSuccess(self):
        rospy.loginfo('PlaceObject executed successfully.')
        self._result.result.success = True
        self._result.result.had_object = True
        self._server.set_succeeded(self._result.result)
        return 1

    def returnFail(self):
        rospy.loginfo('PlaceObject failed. There was no bad exception, just didnt manage to succeed.')
        self._result.result.success = False
        self._result.result.had_object = True
        self._server.set_aborted(self._result.result)
        return -1

    def returnNoObjectToPlace(self):
        rospy.logwarn('We do not seem to have an object. Aborting placing.')
        self._result.result.had_object = False
        self._result.result.success = False
        self._server.set_aborted(self._result.result)
        return -1

    def preemptCallback(self):
        self._moveitInterface.setPreempted(True)

    def returnPreempted(self):
        rospy.loginfo('Preempting PlaceObject action. These people today.... No patience!')
        self._result.result.success = False
        self._server.set_preempted()
        return 0

    def visualizePoses(self, poses):
        # markerArray = visualization_msgs.msg.MarkerArray()
        # for pose in poses:
        #     marker = visualization_msgs.msg.Marker()
        if len(poses) > 0:
            header = Header(stamp=rospy.Time.now(), frame_id=poses[0].header.frame_id)
            visPoses = PoseArray(header=header, poses=map(lambda x: x.pose, poses))
            self._pubPoses.publish(visPoses)

    def doTotePlacing(self, actionGoal):
        try:
            # Get hold of Robot
            self._moveitInterface.acquireLock()
            self._moveitInterface.setPreempted(False)
        except ValueError as e:
            return self.returnError(repr(e))

        try:
            gripperName = self._moveitInterface.getGripperName(actionGoal.arm)
            gripper = self._moveitInterface.getGripper(gripperName)
            if not gripper.isPicking():
                return self.returnNoObjectToPlace()
            (moveResult, trajList) = self._moveitInterface.moveRelative(actionGoal.arm, z=-self._params['lowering_distance'],
                                                                        frameId='tote')
            if moveResult == MoveResult.Success:
                gripper.stop()
                success = True
            else:
                success = False
        except PreemptionException as preemptErr:
            rospy.logwarn('Tote placing was preempted:' + repr(preemptErr))
            return self.returnPreempted()
        except Exception as error:
            return self.returnError('Could not place object because of an error:' + repr(error), error=error)
        finally:
            self._moveitInterface.releaseLock()
        if success:
            return self.returnSuccess()
        return self.returnFail()


    # def sampleApproachPoses(self, binName, arm):
    #     means, devs = 3 * [0.0], 3 * [0.0]
    #     means[0], means[1], means[2] = self._params[binName]['x_mean'], self._params[binName]['y_mean'], self._params[binName]['z_mean']
    #     devs[0], devs[1], devs[2] = self._params[binName]['x_dev'], self._params[binName]['y_dev'], self._params[binName]['z_dev']
    #     poses = []
    #     IPython.embed()
    #     for i in range(self._params['num_approach_poses']):
    #         position = kdl.Vector()
    #         for d in range(3):
    #             position[d] = random.triangular(means[d] - devs[d], means[d] + devs[d])
    #         rotation = kdl.Rotation.RotX(math.pi) * kdl.Rotation.RotY(0.5 * math.pi) * kdl.Rotation.RotZ(self._params[binName]['rot'])
    #         apose = posemath.toMsg(kdl.Frame(rotation, position))
    #         apose = utils.BinPoseMath.getStowingEEFPose(self._moveitInterface, binName, arm, apose)
    #         poses.append(apose)
    #     self.visualizePoses(poses)
    #     (poses, iksolutions) = self._moveitInterface.getFeasiblePoses(poses, arm)
    #     return poses

    def getStowingPose(self, binName, objectName, electric=False):
        stinfo = self._params['stowing_information']
        binType = stinfo['bin_types'][binName]
        if objectName in stinfo:
            poseInfo = stinfo[objectName][binType]
        else:
            poseInfo = stinfo['default_object'][binType]
        position = kdl.Vector(poseInfo['x'], poseInfo['y'], poseInfo['z'])
        if electric:
            rotation = kdl.Rotation.RotY(0.5 * math.pi) * kdl.Rotation.RotZ(poseInfo['rot_electric'])
        else:
            rotation = kdl.Rotation.RotY(0.75 * math.pi) * kdl.Rotation.RotZ(poseInfo['rot'])
        apose = posemath.toMsg(kdl.Frame(rotation, position))
        stampedPose = utils.ArgumentsCollector.makeStamped(apose, binName)
        self.visualizePoses([stampedPose])
        return stampedPose

    def doStowing(self, actionGoal):
        try:
            # Get hold of Robot
            self._moveitInterface.acquireLock()
            self._moveitInterface.setPreempted(False)
        except ValueError as error:
            return self.returnError('Could not place object because some other action has access to the robot.')
        success = False
        try:
            self._moveitInterface.resetConfigurationStack(actionGoal.arm)
            gripperName = self._moveitInterface.getGripperName(actionGoal.arm)
            gripperFrameName = self._moveitInterface.getGripperFrameName(actionGoal.arm)
            gripper = self._moveitInterface.getGripper(gripperName)
            if not gripper.isPicking():
                return self.returnNoObjectToPlace()

            # MOVE TO CORRECT BIN CONFIGURATION
            configName = self._moveitInterface.getConfigurationName(actionGoal.bin_id, 'stowing', 'suction')
            config = self._moveitInterface.getNamedConfiguration(configName, actionGoal.arm)
            if config is None:
                return self.returnError('Could not retrieve stowing configuration for bin ' + actionGoal.bin_id)
            (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(config=config, arm=actionGoal.arm,
                                                                              planningTime=self._params['planning_time_out'])
            if moveArmResult != MoveResult.Success:
                return self.returnError('Could not move arm to pre-bin configuration, aborting.')

            # GET APPROACH POSE
            # stack config
            self._moveitInterface.stackCurrentConfiguration(actionGoal.arm)
            pose = self.getStowingPose(actionGoal.bin_id, actionGoal.targetObject, gripper.isElectric())
            # MOVE TO PREAPPROACH BIN POSE
            (moveArmResult, traj) = self._moveitInterface.moveToPose(pose, actionGoal.arm,
                                                                     planningTime=self._params['planning_time_out'],
                                                                     useRoadmap=False,
                                                                     seedIk=config)
            if moveArmResult != MoveResult.Success:
                self._moveitInterface.retreatConfigurationStack(actionGoal.arm)
                return self.returnError('Could not reach pre-approach pose, aborting')
            else:
                # remove config from the stack again; too risky to keep it
                self._moveitInterface.popConfigurationStack(actionGoal.arm)
            # NOW JUST GO INSIDE OF THE BIN AS FAR AS POSSIBLE
            self._moveitInterface.sleep(1.0)
            # stack approach config
            self._moveitInterface.stackCurrentConfiguration(actionGoal.arm)
            (moveArmResult, traj) = self._moveitInterface.moveRelative(arm=actionGoal.arm,
                                                                       x=self._params['stowing_distance'],
                                                                       frameId=actionGoal.bin_id,
                                                                       interpolate=True,
                                                                       mustReachGoal=False)
            self._moveitInterface.sleep(1.0)
            eefPoseInBin = self._moveitInterface.getTransform(gripperFrameName, base_frame=actionGoal.bin_id)

            # if we failed to get into the shelf, do not drop it
            if eefPoseInBin.pose.position.x < self._params['safe_stowing_distance']:
                rospy.logwarn('Damn, I did not manage to get far enough into the bin, keeping it')
            else:
                # RELEASE THE OBJECT
                rospy.loginfo('This should work! Lets drop this object here.')
                gripper.stop()

            # MOVE OUT AGAIN
            self._moveitInterface.sleep(1.0)
            if moveArmResult != MoveResult.Success:
                rospy.logwarn('It seems like we collided; using configuration stack to move out!')
                self._moveitInterface.retreatConfigurationStack(actionGoal.arm)
            else:
                # everything is going as planned, so move out in a nice way
                (moveArmResult, traj) = self._moveitInterface.moveRelative(arm=actionGoal.arm,
                                                                           x=-eefPoseInBin.pose.position.x - self._params['stowing_retreat_distance'],
                                                                           frameId=actionGoal.bin_id,
                                                                           interpolate=True)
                if moveArmResult != MoveResult.Success:
                    rospy.logwarn('We are having troubles to move out of the bin. Using configuration stack instead.')
                    self._moveitInterface.retreatConfigurationStack(actionGoal.arm)
                else:
                    # pop the
                    self._moveitInterface.popConfigurationStack(actionGoal.arm)

                (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(config=config,
                                                                                  arm=actionGoal.arm,
                                                                                  planningTime=self._params['planning_time_out'],
                                                                                  useRoadmap=False)

            success = not gripper.isPicking()
            if success:
                rospy.loginfo('We did it! The object should be in the bin now, unless Newton fucked us over.')
        except PreemptionException as preemptErr:
            rospy.logwarn('Stowing was preempted:' + repr(preemptErr))
            return self.returnPreempted()
        except MoveItProblemException as moveitErr:
            return self.returnError('Use MoveIt they said.... it does not work. Something is wrong with MoveIt' + repr(moveitErr))
        except Exception as error:
            return self.returnError('Could not place object because an error occured: ' + repr(error), error=error)
        finally:
            self._moveitInterface.releaseLock()
        if success:
            return self.returnSuccess()
        return self.returnFail()

    def execute_cb(self, actionGoal):
        self.updateParameters()
        # If we have arguments do stuff
        if actionGoal.bin_id == 'tote':
            return self.doTotePlacing(actionGoal)
        return self.doStowing(actionGoal)
