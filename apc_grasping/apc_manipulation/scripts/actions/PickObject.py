#!/usr/bin/env python
"""
    This class implementes a ROSAction server for picking an object.

    @author: Joshua Haustein (haustein@kth.se)
"""

import rospy
import actionlib
from apc_manipulation.msg import ApproachAction, ApproachGoal
from apc_manipulation.msg import RetreatAction, RetreatGoal
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
import utils.ArgumentsCollector
import numpy
import tf.transformations
from std_msgs.msg import Header
import tf
import yaml
import random
import utils.TSRs
import utils.BinPoseMath
from utils.MoveItInterface import MoveResult
from utils.MoveItInterface import PreemptionException, MoveItProblemException
from apc_manipulation.msg import PickObjectAction, PickObjectActionResult
import rospkg
import abc
# import visualization_msgs.msg
import IPython
import sys
import traceback
import PyKDL as kdl
from tf_conversions import posemath
import os.path


class PickDirection:
    Top, Left, Right, Front, Back = ['top', 'left', 'right', 'front', 'back']


class PickingResult:
    Success, ApproachError, PickError = ['Success', 'ApproachError', 'PickError']


class Pick:
    Contact, Approach, PreApproach = ['Contact', 'Approach', 'PreApproach']
    Types = [Contact, Approach, PreApproach]

    def __init__(self, contactPose, approachDir):
        self.contactPose = contactPose
        self.approachPose = None
        self.preApproachPose = None
        self.approachDist = -1.0
        self.preApproachDist = -1.0
        self.approachDir = approachDir

    def getPose(self, typus):
        """ typus can be 'Contact', 'Approach', 'PreApproach' """
        if typus == Pick.Contact:
            return self.contactPose
        elif typus == Pick.Approach:
            return self.approachPose
        elif typus == Pick.PreApproach:
            return self.preApproachPose
        else:
            raise ValueError('Unknown type of pose requested: ' + typus)

    def isValid(self):
        return self.contactPose is not None and \
            self.approachPose is not None and \
            self.preApproachPose is not None and \
            self.approachDir is not None


class PicksContainer:
    def __init__(self, picksLists=None):
        self.topPicks = []
        self.leftPicks = []
        self.rightPicks = []
        self.frontPicks = []
        self.backPicks = []
        if picksLists is not None:
            self.setPickLists(picksLists)

    def getPickLists(self):
        return [self.topPicks, self.leftPicks, self.rightPicks, self.frontPicks, self.backPicks]

    def setPickLists(self, picksList):
        self.topPicks = picksList[0]
        self.leftPicks = picksList[1]
        self.rightPicks = picksList[2]
        self.frontPicks = picksList[3]
        self.backPicks = picksList[4]

    def getTotalNumberPicks(self):
        return len(self.topPicks) + len(self.leftPicks) + len(self.rightPicks) + len(self.frontPicks) + len(self.backPicks)

    def getPicks(self, pickDir):
        if pickDir == PickDirection.Top:
            return self.topPicks
        elif pickDir == PickDirection.Left:
            return self.leftPicks
        elif pickDir == PickDirection.Right:
            return self.rightPicks
        elif pickDir == PickDirection.Front:
            return self.frontPicks
        elif pickDir == PickDirection.Back:
            return self.backPicks

    def setPicks(self, picksList, pickDir):
        if pickDir == PickDirection.Top:
            self.topPicks = picksList
        elif pickDir == PickDirection.Left:
            self.leftPicks = picksList
        elif pickDir == PickDirection.Right:
            self.rightPicks = picksList
        elif pickDir == PickDirection.Front:
            self.frontPicks = picksList
        elif pickDir == PickDirection.Back:
            self.backPicks = picksList


class PickCreator(object):
    def __init__(self, moveItInterface, parentAction):
        self._moveitInterface = moveItInterface
        self._binName = ""
        self._arm = ""
        self._params = {}
        self._objName = ""
        self._picksContainer = None
        self._parentAction = parentAction

    def reconfigure(self, bin_id, arm, objName, elGripper, params):
        self._params = params
        self._elGripper = elGripper
        self._binName = bin_id
        self._arm = arm
        self._objName = objName
        self._picksContainer = None

    def createPicks(self, poses):
        rospy.loginfo('Filtering orientation based')
        self._picksContainer = self.filterGeometryBased(poses)

    def getFeasiblePicks(self, pickDir):
        """ Returns a list of feasible picks for the specified pick direction (respective to the object)"""
        rospy.loginfo('Filtering feasibility based')
        picksToFilter = self._picksContainer.getPicks(pickDir)
        picksToFilter = self.filterFeasibilityBased(picksToFilter, typus='Contact',
                                                    maxChecks=self._params['max_feasibility_checks_per_dir'])
        rospy.loginfo('Computing approach poses')
        picksToFilter = self.computeAndFilterApproachPoses(picksToFilter)
        rospy.loginfo('Computing pre-approach poses')
        picksToFilter = self.computePreApproachPoses(picksToFilter)
        self._picksContainer.setPicks(picksToFilter, pickDir)
        rospy.loginfo('We got ' + str(len(picksToFilter)) + ' picks for pick direction: ' + pickDir)
        return picksToFilter

    def createPicksFromPoses(self, poses, approachDir):
        picks = []
        for pose in poses:
            picks.append(Pick(pose, approachDir))
        return picks

    def filterFeasibilityBased(self, picksList, typus='Contact', limitNum=10, maxChecks=15):
        """ Filters all picks in the given list based on kinematic feasibility
            of the pose of typus (either contact, approach and preapproach).
            @param picks list - a list of picks
            @return a list of picks that contains the subset of picks that are feasible
        """
        feasiblePicks = []
        for pick in picksList[0:min(len(picksList), maxChecks)]:
            if len(feasiblePicks) >= limitNum:
                rospy.logwarn('We have ' + str(len(feasiblePicks)) + ' valid ' + typus + ' poses. We stop checking more as requested!')
                break
            config = self.getSeedConfiguration(pick.approachDir)
            pose = pick.getPose(typus)
            (bFeasible, iksolution) = self._moveitInterface.isPoseFeasible(pose, self._arm, seedConfig=config)
            if bFeasible:
                feasiblePicks.append(pick)

        return feasiblePicks

    def isPositionInCurrentBin(self, position):
        """
            Returns whether the given position is within the bounds of the current bin.
            @param position - a list [x,y,z] in the local bin frame of the position.
            @return true if in range, else false
        """
        binDimensionsMap = self._params['binDimensions']
        if not self._binName in binDimensionsMap:
            binDimensions = binDimensionsMap['bin_default']
            rospy.logwarn('Could not find dimensions for bin ' + self._binName + '. Will use default dimensions.')
        else:
            binDimensions = binDimensionsMap[self._binName]
        inXRange = binDimensions['x_min'] <= position[0] <= binDimensions['x_max']
        inYRange = binDimensions['y_min'] <= position[1] <= binDimensions['y_max']
        inZRange = binDimensions['z_min'] <= position[2] <= binDimensions['z_max']
        return inXRange and inYRange and inZRange

    def filterGeometryBased(self, poses):
        """ Filters the given list of contact poses (list of PoseStamped)
            based on the current bin pose.
            Which poses are filtered out is determined by the filterDirection function.
            @param poses - list of contact poses (PoseStamped)
            @return PicksContainer where the picks are assigned the respective approach direction.
        """
        # binPose = self._moveitInterface.getTransform(self._binName)
        # binPoseAsList = utils.ArgumentsCollector.stampedPoseToList(binPose)
        # binMatrix = tf.transformations.quaternion_matrix(binPoseAsList[3:7])
        xAxisBin = numpy.array([1.0, 0.0, 0.0])  #  binMatrix[0:3, 0]
        yAxisBin = numpy.array([0.0, 1.0, 0.0])  # binMatrix[0:3, 1]
        zAxisBin = numpy.array([0.0, 0.0, 1.0])  #binMatrix[0:3, 2]

        topSampleTuples = []
        leftSampleTuples = []
        rightSampleTuples = []
        backSampleTuples = []
        frontSampleTuples = []
        thetaLimit = 0.25 * math.pi
        for sample in poses:
            self._moveitInterface.checkPreemption()
            pose = self._moveitInterface.transformPose(sample, self._binName)
            pose = utils.ArgumentsCollector.stampedPoseToList(pose)
            poseMatrix = tf.transformations.quaternion_matrix(pose[3:7])
            position = pose[0:3]
            if not self.isPositionInCurrentBin(position):
                continue
            zAxis = poseMatrix[0:3, 2]
            thetaX = numpy.arccos(numpy.dot(xAxisBin, zAxis))
            thetaY = numpy.arccos(numpy.dot(yAxisBin, zAxis))
            thetaZ = numpy.arccos(numpy.dot(zAxisBin, zAxis))

            onBackFace = math.pi - thetaLimit <= thetaX and thetaX <= math.pi + thetaLimit
            onFrontFace = abs(thetaX) < thetaLimit
            onTopFace = math.pi - thetaLimit <= thetaZ and thetaZ <= math.pi + thetaLimit
            onBottomFace = abs(thetaZ) < thetaLimit
            onLeftFace = math.pi - thetaLimit <= thetaY and thetaY <= math.pi + thetaLimit
            onRightFace = abs(thetaY) <= thetaLimit
            if onBottomFace:
                continue
            elif onTopFace:
                if not self.filterDirection(PickDirection.Top, thetaZ - math.pi):
                    topSampleTuples.append((sample, thetaZ - math.pi))
            elif onFrontFace:
                if not self.filterDirection(PickDirection.Front, thetaX):
                    frontSampleTuples.append((sample, thetaX))
            elif onRightFace:
                if not self.filterDirection(PickDirection.Right, thetaY):
                    rightSampleTuples.append((sample, thetaY))
            elif onLeftFace:
                if not self.filterDirection(PickDirection.Left, thetaY - math.pi):
                    leftSampleTuples.append((sample, thetaY - math.pi))
            elif onBackFace:
                if not self.filterDirection(PickDirection.Back, thetaX - math.pi):
                    backSampleTuples.append((sample, thetaX - math.pi))
        topSampleTuples.sort(key=lambda x: abs(x[1]))
        frontSampleTuples.sort(key=lambda x: abs(x[1]))
        rightSampleTuples.sort(key=lambda x: abs(x[1]))
        leftSampleTuples.sort(key=lambda x: abs(x[1]))
        backSampleTuples.sort(key=lambda x: abs(x[1]))
        filteredSamplesTop = map(lambda x: x[0], topSampleTuples)
        filteredSamplesLeft = map(lambda x: x[0], leftSampleTuples)
        filteredSamplesRight = map(lambda x: x[0], rightSampleTuples)
        filteredSamplesFront = map(lambda x: x[0], frontSampleTuples)
        filteredSamplesBack = map(lambda x: x[0], backSampleTuples)
        if not self._elGripper:
            # Fix the orientation of the grasp poses
            gripperFrameName = self._moveitInterface.getGripperFrameName(self._arm)
            gripperBaseFrameName = self._moveitInterface.getGripperBaseFrameName(self._arm)
            objFrameName = self._moveitInterface.getObjectFrameName(self._objName)
            eefPoseInGripperBase = self._moveitInterface.getTransform(gripperFrameName, gripperBaseFrameName)

            def fixOrientation(sampleList):
                self._moveitInterface.checkPreemption()
                return utils.BinPoseMath.optimizeOrientationSamples(moveItInterface=self._moveitInterface,
                                                                    binName=self._binName, eefPoseInGripperBase=eefPoseInGripperBase,
                                                                    goalFrameName=objFrameName, poseSamples=sampleList,
                                                                    noiseStdev=self._params['tsr']['rotz_noise_stdev'],
                                                                    desiredZAxis=self.getGripperBaseZAxisInBin(),
                                                                    numAngles=self._params['tsr']['rotz_num_angles'])
            allFilteredSamples = [filteredSamplesTop, filteredSamplesLeft, filteredSamplesRight, filteredSamplesFront, filteredSamplesBack]
            [filteredSamplesTop, filteredSamplesLeft, filteredSamplesRight, filteredSamplesFront, filteredSamplesBack] = map(fixOrientation, allFilteredSamples)
        # Create picks and put it all in a picks container
        picksContainer = PicksContainer()
        picksContainer.topPicks = self.createPicksFromPoses(filteredSamplesTop, PickDirection.Top)
        picksContainer.leftPicks = self.createPicksFromPoses(filteredSamplesLeft, PickDirection.Left)
        picksContainer.rightPicks = self.createPicksFromPoses(filteredSamplesRight, PickDirection.Right)
        picksContainer.frontPicks = self.createPicksFromPoses(filteredSamplesFront, PickDirection.Front)
        picksContainer.backPicks = self.createPicksFromPoses(filteredSamplesBack, PickDirection.Back)
        return picksContainer

    def computeAndFilterApproachPoses(self, picksList, limitNum=10):
        """ Returns approach poses for the given grasp poses (filtered).
            @param picksList - a list of picks
            @return picks list with approach poses """
        objFrameName = self._moveitInterface.getObjectFrameName(self._objName)
        F_bin_obj = posemath.fromMsg(self._moveitInterface.getObjectTransform(self._objName, base_frame=self._binName).pose)
        offsetMatrix = numpy.eye(4, 4)
        minApproachDist = self._params['min_approach_dist']
        maxApproachDist = self._params['max_approach_dist']
        approachStepSize = self._params['approach_step_size']
        numApproachSteps = max(int(math.floor((maxApproachDist - minApproachDist) / approachStepSize)), 1) + 1
        approachRange = [maxApproachDist - x * approachStepSize for x in range(0, numApproachSteps)]

        filteredPicks = []
        for pick in picksList:
            if len(filteredPicks) >= limitNum:
                rospy.logwarn('We have ' + str(len(filteredPicks)) + ' valid approach poses. We stop checking more as requested!')
                break
            for ad in approachRange:
                self._moveitInterface.checkPreemption()
                offsetMatrix[2, 3] = -ad
                poseInObjFrame = self._moveitInterface.transformPose(pick.contactPose, targetFrame=objFrameName)
                poseAsList = utils.ArgumentsCollector.stampedPoseToList(poseInObjFrame)
                graspMatrix = tf.transformations.quaternion_matrix(poseAsList[3:7])
                graspMatrix[:3, 3] = poseAsList[0:3]
                approachMatrix = numpy.dot(graspMatrix, offsetMatrix)
                quaternion = tf.transformations.quaternion_from_matrix(approachMatrix)
                position = kdl.Vector(*approachMatrix[:3,3])
                position = F_bin_obj * position
                if not self.isPositionInCurrentBin([position[0], position[1], position[2]]):
                    continue
                rosPose = utils.ArgumentsCollector.createROSPose(approachMatrix[:3, 3], quaternion,
                                                                 frame_id=objFrameName)
                pick.approachPose = rosPose
                pick.approachDist = ad
                # Check whether this approach pose is feasible
                pickList = self.filterFeasibilityBased([pick], typus='Approach',
                                                       maxChecks=self._params['max_feasibility_checks_per_dir'])
                if len(pickList) == 1:
                    filteredPicks.append(pick)
                    break
                    # else try a smaller approach distance

        return filteredPicks

    @abc.abstractmethod
    def computePreApproachPoses(self, picksContainer):
        pass

    @abc.abstractmethod
    def getSeedConfiguration(self, dir):
        pass

    @abc.abstractmethod
    def filterDirection(self, direction, theta):
        pass

    @abc.abstractmethod
    def getGripperBaseZAxisInBin(self):
        pass

    @abc.abstractmethod
    def getApproachAxis(self):
        pass


class ShelfBinPickCreator(PickCreator):
    def __init__(self, moveItInterface, parentAction):
        super(ShelfBinPickCreator, self).__init__(moveItInterface, parentAction)

    def getSeedConfiguration(self, approachDir):
        configName = self._moveitInterface.getConfigurationName(self._binName, 'picking', approachDir)
        config = self._moveitInterface.getNamedConfiguration(configName, self._arm)
        if config is None:
            raise ValueError('Could not filter picks based on feasbility, because this config is unknown ' + configName)
        return config

    def getGripperBaseZAxisInBin(self):
        return self._params['tsr']['z_axis_bin_gripper_base']

    def computePreApproachPoses(self, picksList):
        """
            Computes pre approach poses a':
            For each pick in the container with approach pose a, project a into the plane that is parallel
            to the plane spanned by the bin's yz axes and contains the current eef position.
            @return picks where the preapproach poses are all set
        """
        binPose = utils.ArgumentsCollector.stampedPoseToList(self._moveitInterface.getTransform(self._binName))
        binMatrix = tf.transformations.quaternion_matrix(binPose[3:7])
        xAxisBin = binMatrix[0:3, 0]
        preBinPose = utils.ArgumentsCollector.createROSPose(position=[-self._params['pre_approach_bin_distance'], 0, 0],
                                                      rotation=[0, 0, 0, 1], frame_id=self._binName)
        preBinPose = utils.ArgumentsCollector.stampedPoseToList(self._moveitInterface.transformPose(preBinPose, 'base'))
        # eefPose = utils.ArgumentsCollector.stampedPoseToList(self._moveitInterface.getTransform(gripperFrameName))
        # e = numpy.array(eefPose[0:3])
        e = numpy.array(preBinPose[0:3])

        for pick in picksList:
            self._moveitInterface.checkPreemption()
            aPose = pick.approachPose
            aPoseBase = self._moveitInterface.transformPose(aPose, 'base')
            aAsList = utils.ArgumentsCollector.stampedPoseToList(aPoseBase)
            a = numpy.array(aAsList[0:3])
            dist = numpy.dot(a - e, xAxisBin)
            aPrime = a - dist * xAxisBin
            pick.preApproachPose = utils.ArgumentsCollector.createROSPose(position=aPrime, rotation=aAsList[3:7])
            pick.preApproachDist = dist
        return picksList

    def filterDirection(self, direction, theta):
        if direction == PickDirection.Back or direction == PickDirection.Front:
            return True
        if theta > self._params['tsr']['filtering_angle']:
            return True
        return False

    def getApproachAxis(self):
        return [1.0, 0.0, 0.0]


class ToteBinPickCreator(PickCreator):
    def __init__(self, moveItInterface, parentAction):
        super(ToteBinPickCreator, self).__init__(moveItInterface, parentAction)

    def getFeasiblePicks(self, pickDir):
        totePose = self._moveitInterface.getTransform('tote_bottom')
        origZ = totePose.pose.position.z 
        totePose.pose.position.z = totePose.pose.position.z - self._params['tote_bottom_lowering_value']
        self._moveitInterface.moveObject('tote_bottom', totePose)
        result = super(ToteBinPickCreator, self).getFeasiblePicks(self, pickDir)
        totePose.pose.position.z = origZ
        self._moveitInterface.moveObject('tote_bottom', totePose)
        return result

    def getSeedConfiguration(self, approachDir):
        config = self._moveitInterface.getNamedConfiguration(self._binName, self._arm)
        if config is None:
            raise ValueError('Could not filter picks based on feasbility, because the tote configuration is unknown')
        return config

    def getGripperBaseZAxisInBin(self):
        return self._params['tsr']['z_axis_tote_gripper_base']

    def computePreApproachPoses(self, picksList):
        """
            Computes pre approach poses a':
            For each pick in the container with approach pose a, project a into the plane that is parallel
            to the plane spanned by the tote's xy axes and contains the current eef position.
            @return picks where the preapproach poses are all set
        """
        binPose = utils.ArgumentsCollector.stampedPoseToList(self._moveitInterface.getTransform(self._binName))
        binMatrix = tf.transformations.quaternion_matrix(binPose[3:7])
        zAxisTote = binMatrix[0:3, 2]
        preBinPose = utils.ArgumentsCollector.createROSPose(position=[0, 0, self._params['pre_approach_tote_distance']],
                                                      rotation=[0, 0, 0, 1], frame_id=self._binName)
        preBinPose = utils.ArgumentsCollector.stampedPoseToList(self._moveitInterface.transformPose(preBinPose, 'base'))
        e = numpy.array(preBinPose[0:3])
        for pick in picksList:
            self._moveitInterface.checkPreemption()
            aPose = pick.approachPose
            aPoseBase = self._moveitInterface.transformPose(aPose, 'base')
            aAsList = utils.ArgumentsCollector.stampedPoseToList(aPoseBase)
            a = numpy.array(aAsList[0:3])
            dist = numpy.dot(a - e, zAxisTote)
            aPrime = a - dist * zAxisTote
            pick.preApproachPose = utils.ArgumentsCollector.createROSPose(position=aPrime, rotation=aAsList[3:7])
            pick.preApproachDist = dist
        return picksList

    def filterDirection(self, direction, theta):
        if theta > self._params['tsr']['filtering_angle']:
            return True
        return False

    def getApproachAxis(self):
        return [0.0, 0.0, 1.0]


class PickObject(object):
    """ ROSAction server for picking an object. """

    def __init__(self, name, moveitInterface):
        self._server = actionlib.SimpleActionServer(name, PickObjectAction, self.execute_cb, False)
        self._server.register_preempt_callback(self.preemptCallback)
        self._moveitInterface = moveitInterface
        self._tsrCacheSuction = {}
        self._tsrCacheGripper = {}
        self._result = PickObjectActionResult()
        self._parameterRetriever = utils.ArgumentsCollector.ArgumentsCollector('/apc/manipulation/pick_object/params',
                                                      ['min_approach_dist', 'max_approach_dist',
                                                       'approach_step_size', 'sucking_time_out',
                                                       'pick_sample_density', 'planning_time_out',
                                                       'max_feasibility_checks_per_dir', 'push_dist', 'lift_dist',
                                                       'retreat_dist', 'wall_separation_dist', 'retreat_step_size',
                                                       'pre_approach_tote_distance',
                                                       'pre_approach_bin_distance',
                                                       'approach_timeout',
                                                       'wall_separation_timeout',
                                                       'num_discrete_tsrs',
                                                       'binDimensions',
                                                       'tote_bottom_lowering_value',
                                                       'tsr'])
        self._server.start()
        self._pubGraspPoses = rospy.Publisher('grasp_poses', PoseArray, queue_size=1)
        self._pubApproachPoses = rospy.Publisher('grasp_approach_poses', PoseArray, queue_size=1)
        self._pubPreApproachPoses = rospy.Publisher('grasp_pre_approach_poses', PoseArray, queue_size=1)
        self._pubSingleGraspPose = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1)
        self._params = {}
        self._params['min_approach_dist'] = 0.025
        self._params['max_approach_dist'] = 0.04
        self._params['approach_step_size'] = 0.01
        self._params['sucking_time_out'] = 1800
        self._params['pick_sample_density'] = 1 / (1e-2 * 1e-2)
        self._params['planning_time_out'] = 10
        self._params['max_feasibility_checks_per_dir'] = 15
        self._params['push_dist'] = 0.05
        self._params['retreat_dist'] = 0.05
        self._params['lift_dist'] = 0.05
        self._params['wall_separation_dist'] = 0.03
        self._params['retreat_step_size'] = 0.01
        self._params['pre_approach_tote_distance'] = 0.15
        self._params['pre_approach_bin_distance'] = 0.08
        self._params['approach_timeout'] = 10.0
        self._params['wall_separation_timeout'] = 10.0
        self._params['num_discrete_tsrs'] = 20
        self._params['tote_bottom_lowering_value'] = 0.3
        self._params['tsr'] = {'filtering_angle': numpy.pi * 0.5,
                               'z_axis_bin_gripper_base': [1.0, 0.0, 0.0],
                               'rotz_num_angles': 72,
                               'rotz_noise_stdev': 15 * numpy.pi / 180.0,
                               'stow_pos_noise_stdev': 3*[0.0],
                               'stow_rpy_noise_stdev': [15*numpy.pi/180.0, 15*numpy.pi/180.0, 0.0],
                               'min_samples': 200}
        self._params['binDimensions'] = {'bin_default': {'x_min': 0.0, 'x_max': 0.5, 'y_min': -0.1, 'y_max': 0.1,
                                                         'z_min': 0.0,'z_max': 0.2}}

        self.updateParameters()
        self._approach_client = actionlib.SimpleActionClient('/apc/manipulation/approach', ApproachAction)
        rospy.loginfo('[apc_manipulation/PickObject]: waiting for approach action server')
        self._approach_client.wait_for_server()
        self._retreat_client = actionlib.SimpleActionClient('/apc/manipulation/retreat', RetreatAction)
        rospy.loginfo('[apc_manipulation/PickObject]: waiting for retreat action server')
        self._retreat_client.wait_for_server()
        self._approach_success = False
        self._retreat_success = False
        self._shelfPickCreator = ShelfBinPickCreator(self._moveitInterface, self)
        self._totePickCreator = ToteBinPickCreator(self._moveitInterface, self)
        self._pickCreator = self._shelfPickCreator
        # self._markerPublisher = rospy.Publisher(
        #     '/grasp_poses', visualization_msgs.msg.MarkerArray, queue_size=1)

    def updateParameters(self):
        params = self._parameterRetriever.getArguments(True)
        for key in params.keys():
            self._params[key] = params[key]

    def preemptCallback(self):
        self._moveitInterface.setPreempted(True)
        self._approach_client.cancel_all_goals()

    def returnError(self, errorMsg, sucking=False, error=None):
        rospy.logerr(errorMsg)
        if error is not None:
            stackMsg = traceback.format_exc(error)
            rospy.logerr(stackMsg)
        if sucking:
            rospy.logwarn('Although we had an error, we are sucking sth. We therefore return success.')
            return self.returnSuccess()
        self._result.result.success = False
        self._server.set_aborted(self._result.result)
        return -1

    def returnSuccess(self):
        rospy.loginfo('PickArm executed successfully.')
        self._result.result.success = True
        self._server.set_succeeded(self._result.result)
        return 1

    def returnFail(self):
        rospy.loginfo('PickObject executed without any errors, but we could not pick an object.')
        self._result.result.success = False
        self._server.set_aborted(self._result.result)
        return -1

    def returnPreempted(self):
        rospy.loginfo('Preempting this action. These people today.... No patience!')
        self._result.result.success = False
        self._server.set_preempted()
        return 0

    def loadTSRs(self, objname, tsr_type='suction', task='pick'):  # add tsr_type
        """ Attempts to load the TSRs for the given object.
            If successful, it returns a tsrList that contains all TSRs
            @param objname - string object name
            @param tsr_type - 'suction' or 'gripper'
            @param task - 'pick' or 'stow'
            @return tsrList """
        if tsr_type == 'suction':
            tsrCache = self._tsrCacheSuction

        elif tsr_type == 'gripper':
            tsrCache = self._tsrCacheGripper

        if objname not in tsrCache:

            try:
                apc_manipulationDir = rospkg.RosPack().get_path('apc_manipulation') + '/data/tsrs/'
                foundTSR = False

                filepath = apc_manipulationDir + task + '/' + tsr_type + '/' + objname + '.tsrs'
                if os.path.isfile(filepath):
                    foundTSR = True

                if not foundTSR:
                    rospy.logerr('Could not load TSR for: ' + objname + ' Loading default TSRs')
                    filepath = apc_manipulationDir + 'default.tsrs'

                tsrFile = open(filepath, 'r')
                tsrs = yaml.load(tsrFile)
                tsrFile.close()
                # volumes = [x.getVolume() for x in tsrs]
                # accVolume = sum(volumes)
                # volumes = map(lambda x: 1.0 / accVolume * x, volumes)
                # volumes = len(tsrs) * [1.0 / len(tsrs)]

                tsrCache[objname] = tsrs
            except rospy.ROSException as roserr:
                rospy.logerr('We have some ROS exception in loadTSRs: ' + repr(roserr))
                return None
            except IOError as ioerr:
                rospy.logerr('Could not load TSRs for object ' +
                             objname + ' from file: \n' + ioerr.message)
                return None
            except KeyError as keyError:
                rospy.logerr('Could not retrieve TSR file path for object ' +
                             objname + ' from parameter server: \n' + keyError.message)
                return None
        return tsrCache[objname]

    def sampleTSR(self, tsr, numSamples, posTolerance, rotTolerance, objFrameName):
        """ Samples a single TSR.
            @param numSamples - number of samples to draw.
            @param posTolerance - position tolerance (minimal distance to the boundary)
            @param rotTolerance - rotation tolerance (minimal distance to the boundary)
            @param objFrameName - name of the object frame
            @return (samples, posTolerance, rotTolerance) - where samples is a list of sampled
             poses (PoseStamped), posTolerance is the minimal distance that any pose is away from position
             boundaries, rotTolerance is the minimal distance that any pose is away from rotation
             boundaries. """
        if not isinstance(tsr, utils.TSRs.BoxTSR):
            raise NotImplementedError('For now only box TSRs are supported.')
        samples = []
        minRotTolerance = rotTolerance
        minPosTolerance = posTolerance
        for i in range(numSamples):
            self._moveitInterface.checkPreemption()
            sample, tPosT, tRotT = tsr.drawSample(posPadding=posTolerance, rotPadding=rotTolerance)
            position = sample[0:3]
            orientation = tf.transformations.quaternion_from_euler(sample[3], sample[4], sample[5], 'szyx')
            samples.append(utils.ArgumentsCollector.createROSPose(position, orientation, frame_id=objFrameName))
            minRotTolerance = min(minRotTolerance, tRotT)
            minPosTolerance = min(minPosTolerance, tPosT)
        return samples, minPosTolerance, minRotTolerance

    def reduceDiscreteTSRs(self, discreteTSRs, samplingDensity):
        '''
        Reduce number of discrete TSRs according to sampling density
        @param discreteTSRs:
        @param samplingDensity:
        @return:
        '''

        # discrete_tsrs_area = sum(map(lambda x: x.getArea(), discreteTSRs))
        # num_samples = int(discrete_tsrs_area * samplingDensity)
        num_samples = self._params['num_discrete_tsrs']

        if num_samples <= 0:
            return []

        if len(discreteTSRs) < num_samples:
            return discreteTSRs

        # randomly pick num_samples discrete_tsrs
        output_discrete_tsrs = random.sample(discreteTSRs, num_samples)
        return output_discrete_tsrs

    def classifyTSRs(self, tsrList):
        '''
        Separates discrete TSRs (box with zero volume) from surface TSRs
        @param tsrList: utils.TSRs.BoxTSR()
        @return: (discrete_tsrs, surface_tsrs)
        '''
        discrete_tsrs = []
        surface_tsrs = []

        for tsr in tsrList:
            if tsr.isDiscrete():
                discrete_tsrs.append(tsr)
            else:
                surface_tsrs.append(tsr)

        return [discrete_tsrs, surface_tsrs]

    def sampleTSRs(self, tsrList, objFrameName, samplingDensity=(1 / (1e-2 * 1e-2)), posTolerance=0.03, rotTolerance=0.05,
                   posStdev=numpy.zeros(3), rpyStdev=numpy.zeros(3)):
        """ Sample all TSRs according to weight distribution and saves
            the picks in self._pickCreator.
            @param tsrList - list of TSRs to sample.
            @param objFrameName - the name of the object frame
            @param samplingDensity - density for drawing samples from TSRs.
            @param posTolerance - desired position tolerance
            @param rotTolerance - desired rotation tolerance
            @param posStdev - stdev for gaussian noise added to TSR poses
            @param rpyStdev - stdev for gaussian noise added to TSR poses
            @return (posTolerance, rotTolerance) -  posTolerance the achieved position tolerance, rotTolerance the achieved rotation tolerance.
        """
        minRotTolerance = rotTolerance
        minPosTolerance = posTolerance
        # additionalSamples = 0
        discrete_tsrs, surface_tsrs = self.classifyTSRs(tsrList)
        discrete_tsrs = self.reduceDiscreteTSRs(discreteTSRs=discrete_tsrs,
                                                samplingDensity=samplingDensity)
        tsr_list_reduced = discrete_tsrs + surface_tsrs
        allSamples = []
        for t in range(len(tsr_list_reduced)):
            self._moveitInterface.checkPreemption()
            numSamplesTSR = max(1, int(tsr_list_reduced[t].getArea() * samplingDensity))
            # First sample the TSR
            tsrSamples, tposT, trotT = self.sampleTSR(
                tsr_list_reduced[t], numSamplesTSR, posTolerance, rotTolerance, objFrameName)

            minPosTolerance = min(tposT, minPosTolerance)
            minRotTolerance = min(trotT, minRotTolerance)
            allSamples.extend(tsrSamples)
            # additionalSamples = max(numSamplesTSR - len(tsrTopPicks) - len(tsrLeftPicks) - len(tsrRightPicks), 0)

        while len(allSamples) < self._params['tsr']['min_samples']:
            for t in range(len(tsr_list_reduced)):
                self._moveitInterface.checkPreemption()
                numSamplesTSR = max(1, int(tsr_list_reduced[t].getArea() * samplingDensity))
                # First sample the TSR
                tsrSamples, tposT, trotT = self.sampleTSR(
                    tsr_list_reduced[t], numSamplesTSR, posTolerance, rotTolerance, objFrameName)

                minPosTolerance = min(tposT, minPosTolerance)
                minRotTolerance = min(trotT, minRotTolerance)
                allSamples.extend(tsrSamples)
                # additionalSamples = max(numSamplesTSR - len(tsrTopPicks) - len(tsrLeftPicks) - len(tsrRightPicks), 0)

        allSamplesWithNoise = self.addNoiseTSRs(tsrList=allSamples, posStdev=posStdev, rpyStdev=rpyStdev)

        # Now filter out TSRs that are pointless to attempt reaching (bottom face, not feasible etc)
        # Also, split samples based on approach direction
        self._pickCreator.createPicks(allSamplesWithNoise)
        return minPosTolerance, minRotTolerance

    def addNoiseTSRs(self, tsrList, posStdev, rpyStdev):
        '''
        Adds position + orientation noise to TSR samples
        @param tsrList:
        @param posStdev: [x, y, z] standard deviation for position noise (numpy.array)
        @param rpyStdev: [roll, pitch, yaw] standard deviation for orientation noise (numpy.array)
        @return: tsrList with added noise on each sample
        '''

        tsrListWithNoise = []
        if numpy.linalg.norm(posStdev) < 1e-6 and numpy.linalg.norm(rpyStdev) < 1e-6:
            tsrListWithNoise.extend(tsrList)
            return tsrListWithNoise

        for tsr in tsrList:
            p = kdl.Vector()
            rpy = 3*[0.0]

            for i in range(3):
                if posStdev[i] != 0.0:
                    p[i] = numpy.random.normal(0.0, posStdev[i])

                if rpy[i] != 0.0:
                    rpy[i] = numpy.random.normal(0.0, rpyStdev[i])

            F = kdl.Frame(kdl.Rotation.RPY(*rpy), p)
            F_tsr = posemath.fromMsg(tsr.pose)

            tsrWithNoise = PoseStamped()
            tsrWithNoise.header = tsr.header
            tsrWithNoise.pose = posemath.toMsg(F_tsr * F)

            tsrListWithNoise.append(tsrWithNoise)

        return tsrListWithNoise

    def visualizePoses(self, poses, typus=None):
        # markerArray = visualization_msgs.msg.MarkerArray()
        # for pose in poses:
        #     marker = visualization_msgs.msg.Marker()
        if len(poses) > 0:
            if typus != 'all':
                vposes = poses
                if typus is not None:
                    vposes = map(lambda x: x.getPose(typus), poses)
                header = Header(stamp=rospy.Time.now(), frame_id=vposes[0].header.frame_id)
                visPoses = PoseArray(header=header, poses=map(lambda x: x.pose, vposes))
                self._pubSingleGraspPose.publish(vposes[0])
                self._pubGraspPoses.publish(visPoses)
            else:
                vPoseList = []
                for typus in Pick.Types:
                    vposes = map(lambda x: x.getPose(typus), poses)
                    header = Header(stamp=rospy.Time.now(), frame_id=vposes[0].header.frame_id)
                    visPoses = PoseArray(header=header, poses=map(lambda x: x.pose, vposes))
                    vPoseList.append(visPoses)
                self._pubGraspPoses.publish(vPoseList[0])
                self._pubApproachPoses.publish(vPoseList[1])
                self._pubPreApproachPoses.publish(vPoseList[2])

    def attemptPicking(self, gripper, picks, binName, armName, approachDir):
        rospy.loginfo('Attempting to pick from ' + approachDir)
        self.visualizePoses(picks, typus='all')
        stepSize = self._params['retreat_step_size']
        self._moveitInterface.resetConfigurationStack(armName)
        configName = self._moveitInterface.getConfigurationName(binName, 'picking', approachDir)
        startConfig = self._moveitInterface.getNamedConfiguration(configName, armName)
        (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(config=startConfig, arm=armName)
        gripperFrameName = self._moveitInterface.getGripperFrameName(armName)
        if moveArmResult != MoveResult.Success:
            rospy.logerr('Could not move arm to start configuration, aborting.')
            return PickingResult.ApproachError

        # PRE-APPROACH
        selectedPick = None
        approachTrajs = [[], []]
        for pick in picks:
            approachTrajs = [[], []]
            if pick.isValid():
                (moveArmResult, traj) = self._moveitInterface.moveToPose(pick.preApproachPose, armName,
                                                                         planningTime=self._params['planning_time_out'],
                                                                         useRoadmap=False, execute=False)
                if moveArmResult != MoveResult.Success:
                    continue
                approachTrajs[0] = traj
                preApproachConfig = utils.ArgumentsCollector.dictFromJointState(traj[-1].joint_trajectory.joint_names,
                                                                                traj[-1].joint_trajectory.points[-1].positions)
                # APPROACH
                approachAxis = self._pickCreator.getApproachAxis()
                approachAxis = map(lambda x: x * pick.preApproachDist, approachAxis)
                (moveArmResult, traj) = self._moveitInterface.moveRelative(armName, x=approachAxis[0],
                                                                           y=approachAxis[1], z=approachAxis[2],
                                                                           frameId=binName, interpolate=True,
                                                                           stepSize=stepSize, startConfig=preApproachConfig,
                                                                           execute=False)
                if moveArmResult == MoveResult.Success:
                    rospy.loginfo('We found an approach pose that seems reachable.')
                    approachTrajs[1] = traj
                    selectedPick = pick
                    break

        # Do we have a pick that seems to be reachable?
        if selectedPick is None:
            rospy.logerr('Could not move arm to any pre-approach pose, aborting.')
            return PickingResult.ApproachError

        # If so, execute the first planned trajectory sequence
        moveArmResult = self._moveitInterface.executeTraj(armName, approachTrajs[0])
        if moveArmResult != MoveResult.Success:
            rospy.logerr('We could not move to the pre-approach pose we selected. Aborting.')
            return PickingResult.ApproachError
        # This should usually work, let us now move towards the approach pose
        self._moveitInterface.stackCurrentConfiguration(armName)
        moveArmResult = self._moveitInterface.executeTraj(armName, approachTrajs[1])
        if moveArmResult != MoveResult.Success:
            rospy.logerr('We could not move to the approach pose we selected. Aborting.')
            self._retreat(armName, startConfig)
            return PickingResult.PickError
        # PICK
        rospy.loginfo('Reached approach pose. About to pick...')
        # save this configuration for emergency retreat
        self._moveitInterface.stackCurrentConfiguration(armName)
        # also save this configuration for normal retreat
        preContactConfig = self._moveitInterface.getCurrentConfiguration(armName)
        if gripper.isElectric():
            rospy.loginfo('Oh boy, oh boy, here we go gripping again.')
            distanceToGo = selectedPick.approachDist + self._params['push_dist_electric']
            (moveArmResult, traj) = self._moveitInterface.moveRelative(armName, z=distanceToGo,
                                                                       interpolate=True,
                                                                       stepSize=stepSize,
                                                                       mustReachGoal=False,
                                                                       controlMode='position')
            gripper.pick()
            if moveArmResult != MoveResult.Success:
                rospy.logerr('Could not pick object, aborting.')
                self._retreat(armName, startConfig)
                return PickingResult.PickError

            # POST PICKING
            rospy.loginfo('Lifting the object')
            self._moveitInterface.moveToConfiguration(preContactConfig, armName, noPlanning=True)
            self._moveitInterface.popConfigurationStack(armName)
        else:
            rospy.loginfo('Oh boy, oh boy, here we go sucking again.')
            approachGoal = ApproachGoal()
            approachGoal.arm = armName
            # TODO: we could compute the distance to the wall/ground here
            approachGoal.max_distance = selectedPick.approachDist + self._params['push_dist']
            self._approach_client.send_goal(approachGoal)
            success = self._approach_client.wait_for_result(rospy.Duration(self._params['approach_timeout']))
            if not success:
                rospy.logerr('Could not suck object, aborting.')
                self._retreat(armName, startConfig)
                return PickingResult.PickError

            # POST PICKING
            rospy.loginfo('Moving the object away from the wall/floor.')
            retreatGoal = RetreatGoal()
            retreatGoal.arm = armName
            # TODO: idea use the distance given in the feedback of the approach action instead of using this:
            retreatGoal.max_distance = min(abs(selectedPick.approachDist), self._params['wall_separation_dist'])
            self._retreat_client.send_goal(retreatGoal)
            success = self._retreat_client.wait_for_result(rospy.Duration(self._params['wall_separation_timeout']))
            if not success:
                rospy.logerr('Could not lift/retreat from the wall, aborting. Sucking the shelf?')
                gripper.stop()
                self._retreat(armName, startConfig)
                return PickingResult.PickError
            self._moveitInterface.popConfigurationStack(armName)

        if selectedPick.approachDir !=PickDirection.Top:
            (moveArmResult, traj) = self._moveitInterface.moveRelative(armName, z=self._params['lift_dist'],
                                                                       frameId=binName, interpolate=True,
                                                                       stepSize=stepSize, mustReachGoal=False,
                                                                       controlMode='position')
            # we do not care about the result here

        self._moveitInterface.sleep(1.0)
        success = gripper.isPicking()
        rospy.loginfo('Ready to move out. Vacuum: ' + str(success))
        if success:
            rospy.loginfo('We have an object, lets get the hell outa here!!!!')
            approachAxis = self._pickCreator.getApproachAxis()
            approachAxis = map(lambda x: x * (-1.0) * (selectedPick.preApproachDist + numpy.sign(selectedPick.preApproachDist) * self._params['retreat_dist']), approachAxis)
            (moveArmResult, traj) = self._moveitInterface.moveRelative(armName, x=approachAxis[0],
                                                                       y=approachAxis[1], z=approachAxis[2],
                                                                       frameId=binName, interpolate=True,
                                                                       stepSize=stepSize, mustReachGoal=False)
            if moveArmResult != MoveResult.Success:
                rospy.logerr('Could not move arm back to pre-approach pose, aborting.')
                # gripper.stop()
                self._retreat(armName, startConfig)
                return PickingResult.PickError
            self._moveitInterface.popConfigurationStack(armName)
            (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(config=startConfig,
                                                                              arm=armName,
                                                                              useRoadmap=False)
            if moveArmResult != MoveResult.Success:
                rospy.logerr('Could not move arm to post-picking pose, aborting.')
                self._retreat(armName, startConfig)
                return PickingResult.PickError
            self._moveitInterface.sleep(1.0)
            success = gripper.isPicking()
            rospy.loginfo('We made it, it was tough, but we made it. Did we pick sth: ' + str(success))
            if success:
                return PickingResult.Success
            return PickingResult.PickError
        rospy.logerr('Could not pick object. We failed at sucking/grasping it :(')
        self._retreat(armName, startConfig)
        return PickingResult.PickError

    def picking(self, actionGoal):
        self.updateParameters()
        # First, get hold of robot
        try:
            self._moveitInterface.acquireLock()
            self._moveitInterface.setPreempted(False)
        except ValueError as error:
            return self.returnError('Failed to get access to robot: ' + repr(error))

        # At this point we have access to the Robot
        try:
            # Get frames and frame names
            gripperName = self._moveitInterface.getGripperName(actionGoal.arm)
            objectFrameName = self._moveitInterface.getObjectFrameName(actionGoal.targetObject)
            gripper = self._moveitInterface.getGripper(gripperName)

            # Set up MoveIt planning scene
            objectsToAdd = [x for x in actionGoal.obstacles]
            # objectsToAdd.append(actionGoal.targetObject)
            self._moveitInterface.addObjectsToPlanningScene(objectsToAdd)

            # Sample TSR
            if actionGoal.bin_id == 'tote':
                task = 'stow'
            else:
                task = 'pick'

            tsrObjectName = actionGoal.targetObject
            if not actionGoal.is_tf_reliable:
                if task == 'stow':
                    tsrObjectName = 'rgb_stowing'
                else:
                    tsrObjectName = 'rgb_picking'
            tsrType = 'suction'
            if gripper.isElectric():
                tsrType = 'gripper'
            tsrList = self.loadTSRs(objname=tsrObjectName, tsr_type=tsrType, task=task)
            rospy.loginfo('Loaded TSR for object ' + tsrObjectName)
            # select pick creator based on whether we are picking from the shelf or the tote
            stow_task = (actionGoal.bin_id == 'tote')
            if stow_task:
                self._pickCreator = self._totePickCreator
            else:
                self._pickCreator = self._shelfPickCreator

            self._pickCreator.reconfigure(actionGoal.bin_id, actionGoal.arm, actionGoal.targetObject,
                                          gripper.isElectric(), self._params)

            # add noise to TSRs if we are doing stowing
            if stow_task and not gripper.isElectric():
                posTol, rotTol = self.sampleTSRs(tsrList=tsrList,
                                                 objFrameName=objectFrameName,
                                                 samplingDensity=self._params['pick_sample_density'],
                                                 posStdev=numpy.array(self._params['tsr']['stow_pos_noise_stdev']),
                                                 rpyStdev=numpy.array(self._params['tsr']['stow_rpy_noise_stdev']))

            else:
                posTol, rotTol = self.sampleTSRs(tsrList=tsrList,
                                                 objFrameName=objectFrameName,
                                                 samplingDensity=self._params['pick_sample_density'])

            rospy.loginfo('Sampled TSRs for ' + actionGoal.targetObject)
            # rospy.loginfo('We have ' + str(len(picksContainer.topPicks)) + ' top, ' +
            #               str(len(picksContainer.leftPicks)) + ' left, ' + str(len(picksContainer.rightPicks)) +
            #               ' right, ' + str(len(picksContainer.frontPicks)) + ' front, ' +
            #               str(len(picksContainer.backPicks)) + ' back picking poses.')
            # totalNumSamples = picksContainer.getTotalNumberPicks()
            # if totalNumSamples == 0:
            #     rospy.logerr('We do not have any picking poses. Aborting!')
            #     return self.returnFail()

            pickingResult = PickingResult.ApproachError
            topPicks = self._pickCreator.getFeasiblePicks(PickDirection.Top)
            if len(topPicks) > 0:  # TOP PICKING
                pickingResult = self.attemptPicking(gripper, topPicks, actionGoal.bin_id, actionGoal.arm, PickDirection.Top)
            if pickingResult == PickingResult.ApproachError:
                leftPicks = self._pickCreator.getFeasiblePicks(PickDirection.Left)
                if len(leftPicks) > 0:  # LEFT PICKING
                    pickingResult = self.attemptPicking(gripper, leftPicks, actionGoal.bin_id, actionGoal.arm, PickDirection.Left)
            if pickingResult == PickingResult.ApproachError:
                rightPicks = self._pickCreator.getFeasiblePicks(PickDirection.Right)
                if len(rightPicks) > 0:  # RIGHT PICKING
                    pickingResult = self.attemptPicking(gripper, rightPicks, actionGoal.bin_id, actionGoal.arm, PickDirection.Right)
            if pickingResult == PickingResult.ApproachError:
                frontPicks = self._pickCreator.getFeasiblePicks(PickDirection.Front)
                if len(frontPicks) > 0:  #  FRONT PICKING
                    pickingResult = self.attemptPicking(gripper, frontPicks, actionGoal.bin_id, actionGoal.arm, PickDirection.Front)
            if pickingResult == PickingResult.ApproachError:
                backPicks = self._pickCreator.getFeasiblePicks(PickDirection.Back)
                if len(backPicks) > 0:  #  BACK PICKING
                    pickingResult = self.attemptPicking(gripper, backPicks, actionGoal.bin_id, actionGoal.arm, PickDirection.Back)
        except PreemptionException as preemptErr:
            rospy.logwarn('Pick action was preempted. Message: ' + repr(preemptErr))
            return self.returnPreempted()
        except MoveItProblemException as moveitErr:
            return self.returnError('Use MoveIt they said.... it does not work. Something is wrong with MoveIt' + repr(moveitErr),
                                    gripper.isPicking())
        except Exception as error:
            return self.returnError('There was an error while picking an object: ' + repr(error), gripper.isPicking(), error)
        finally:
            if not gripper.isPicking():
                gripper.stop()
            self._moveitInterface.releaseLock()
        if pickingResult == PickingResult.Success or gripper.isPicking():
            return self.returnSuccess()
        return self.returnFail()

    def execute_cb(self, actionGoal):
        return self.picking(actionGoal)

    def _retreat(self, armName, config):
        self._moveitInterface.retreatConfigurationStack(armName)
        (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(config=config, arm=armName, useRoadmap=False)
        if moveArmResult != MoveResult.Success:
            rospy.logerr('We are in serious trouble! We can not retreat!')
            raise RuntimeError('We are doomed. How do we fix this????')
        #     (moveArmResult, traj) = self._moveitInterface.moveToConfiguration(config=config, arm=armName)
        #     if moveArmResult != MoveResult.Success:
