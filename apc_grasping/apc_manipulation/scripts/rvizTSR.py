#!/usr/bin/python

#   rvizTSR.py
#
#   Created on: June 15, 2016
#   Authors:   Francisco Vina
#              fevb@kth.se
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
import tf
import argparse
import utils.TSRs
import utils.ArgumentsCollector
import rospkg
import yaml
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header
import numpy
import copy
from tf_conversions import posemath
import PyKDL as kdl
import os.path


class RvizTSRVisualizer:

    def __init__(self):
        '''
        Class for publishing TSR poses to RVIZ
        '''
        self._tsrCacheSuction = {}
        self._tsrCacheGripper = {}
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._pubGraspPoses = rospy.Publisher('grasp_poses', PoseArray, queue_size=1)
        self._pubGraspPosesZAxis = rospy.Publisher('grasp_poses_z_axis', PoseArray, queue_size=1)
        self._pubSingleGraspPose = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1)

    def loadTSR(self, objname, tsr_type='suction', task='pick'):
        """ Attempts to load the TSRs for the given object.
            If successful, it returns a tuple (tsrList, tsrVolumes), where
            tsrList contains all TSRs and tsrVolumes the normalized volumes of
            the TSRs.
            @param objname - string object name
            @return (tsrList, tsrVolumes) """

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

                #filepath = + objname
                tsrFile = open(filepath, 'r')
                tsrs = yaml.load(tsrFile)
                tsrFile.close()
                # volumes = [x.getVolume() for x in tsrs]
                # accVolume = sum(volumes)
                # volumes = map(lambda x: 1.0 / accVolume * x, volumes)
                volumes = len(tsrs) * [1.0 / len(tsrs)]

                tsrCache[objname] = (tsrs, volumes)

            except IOError as ioerr:
                rospy.logerr('Could not load TSRs for object ' +
                             objname + ' from file: \n' + ioerr.message)
                return None
            except KeyError as keyError:
                rospy.logerr('Could not retrieve TSR file path for object ' +
                             objname + ' from parameter server: \n' + keyError.message)
                return None
        return tsrCache[objname]


    def sampleTSR(self, tsr, numSamples, posTolerance, rotTolerance, objname):
        """ Samples a single TSR.
            @param numSamples - number of samples to draw.
            @param posTolerance - position tolerance (minimal distance to the boundary)
            @param rotTolerance - rotation tolerance (minimal distance to the boundary)
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
            sample, tPosT, tRotT = tsr.drawSample(posPadding=posTolerance, rotPadding=rotTolerance)
            position = sample[0:3]
            orientation = tf.transformations.quaternion_from_euler(sample[3], sample[4], sample[5], 'szyx')
            samples.append(utils.ArgumentsCollector.createROSPose(position, orientation, frame_id=objname))
            minRotTolerance = min(minRotTolerance, tRotT)
            minPosTolerance = min(minPosTolerance, tPosT)
        return samples, minPosTolerance, minRotTolerance

    def reduceDiscreteTSRs(self, discreteTSRs, samplingDensity):

        discrete_tsrs_area = sum(map(lambda x: x.getArea(), discreteTSRs))
        num_samples = int(discrete_tsrs_area*samplingDensity)

        if num_samples <= 0:
            return []

        if len(discreteTSRs) < num_samples:
            return discreteTSRs

        # randomly pick num_samples discrete_tsrs
        pick_tsrs = numpy.array([])

        pick_tsrs = numpy.random.choice(numpy.arange(len(discreteTSRs)), num_samples, replace=False)

        output_discrete_tsrs = map(lambda x: discreteTSRs[x], pick_tsrs)

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


    def sampleTSRs(self, tsrList, tsrWeights, objname, samplingDensity=(1/(1e-2*1e-2)), posTolerance=0.01, rotTolerance=0.05):

        topSamples, leftSamples, rightSamples = [], [], []
        samples = []
        minRotTolerance = rotTolerance
        minPosTolerance = posTolerance
        additionalSamples = 0

        discrete_tsrs, surface_tsrs = self.classifyTSRs(tsrList)
        discrete_tsrs = self.reduceDiscreteTSRs(discreteTSRs=discrete_tsrs,
                                                samplingDensity=samplingDensity)
        tsr_list_reduced = discrete_tsrs + surface_tsrs
        
        for t in range(len(tsr_list_reduced)):
            numSamplesTSR = max(1, int(tsr_list_reduced[t].getArea()*samplingDensity))
            # First sample the TSR
            tsrSamples, tposT, trotT = self.sampleTSR(
                tsr_list_reduced[t], numSamplesTSR, posTolerance, rotTolerance, objname)
            #
            # # Now filter out samples that are not kinematically feasible
            # (tsrTopSamples, tsrLeftSamples, tsrRightSamples) = self.filterFeasibilityBased(tsrTopSamples,
            #

            samples.extend(tsrSamples)
            additionalSamples = max(numSamplesTSR - len(tsrSamples), 0)
        return samples


    def computeApproachPoses(self, graspPoses, approachDist, objectFrameName):
        """ Returns approach poses for the given grasp poses (filtered).
            @param graspPosesTop - a list of poses (PoseStamped)
            @param graspPosesLeft - a list of poses (PoseStamped)
            @param graspPosesRight - a list of poses (PoseStamped)
            @param binName - name of the bin
            @param armName - name of the arm
            @param approachDist - distance to offset along eef z axis
            @param objectFrameName - name of the object frame
            @return list of approach poses as PoseStamped """
        offsetMatrix = numpy.eye(4, 4)
        offsetMatrix[2, 3] = -approachDist
        approachPoses = []
        for graspPose in graspPoses:
            poseInObjFrame = copy.deepcopy(graspPose)
            #poseInObjFrame = self._moveitInterface.transformPose(graspPose, targetFrame=objectFrameName)
            poseAsList = utils.ArgumentsCollector.stampedPoseToList(poseInObjFrame)
            graspMatrix = tf.transformations.quaternion_matrix(poseAsList[3:7])
            graspMatrix[:3, 3] = poseAsList[0:3]
            approachMatrix = numpy.dot(graspMatrix, offsetMatrix)
            quaternion = tf.transformations.quaternion_from_matrix(approachMatrix)
            rosPose = utils.ArgumentsCollector.createROSPose(
                approachMatrix[:3, 3], quaternion, frame_id=objectFrameName)
            approachPoses.append(rosPose)
        # approachPoses.append(([:3, 3], quaternion))
            # approachPoses.append(approachMatrix)

        return approachPoses

    def transformPoseZaxis(self, pose):
        '''
        Transforms a pose so that X axis points towars original Z axis.
        This way its possible to view in RVIZ where Z-axis points at
        @param pose:
        @return:
        '''

        F = posemath.fromMsg(pose)
        F_new = F*kdl.Frame(kdl.Rotation.RotY(-numpy.pi*0.5), kdl.Vector())

        return posemath.toMsg(F_new)



    def visualizeTSRs(self, objname, tsr_type, task):
        r = rospy.Rate(10.0)

        tsr_list, tsr_weights = self.loadTSR(objname=objname, tsr_type=tsr_type, task=task)
        tsr_samples = self.sampleTSRs(tsrList=tsr_list, tsrWeights=tsr_weights, objname=args.objname,
                                      posTolerance=0.000001, rotTolerance=0.00001)

        poses = self.computeApproachPoses(tsr_samples, 0.0, objname)

        while not rospy.is_shutdown():
            if len(poses) > 0:
                header = Header(stamp=rospy.Time.now(), frame_id=poses[0].header.frame_id)
                visPoses = PoseArray(header=header, poses=map(lambda x: x.pose, poses))
                visPosesZAxis = PoseArray(header=header, poses=map(lambda x: self.transformPoseZaxis(x.pose), poses))
                self._pubSingleGraspPose.publish(poses[0])
                self._pubGraspPosesZAxis.publish(visPosesZAxis)
                self._pubGraspPoses.publish(visPoses)

            self._tf_broadcaster.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(),
                                                poses[0].header.frame_id, 'map')


            r.sleep()

if __name__=='__main__':
    rospy.init_node('rviz_tsr')
    parser = argparse.ArgumentParser(description='Visualize TSRs!')
    parser.add_argument('objname',type=str,
                        help='Name of the object.')
    parser.add_argument('tsr_type', type=str, default='suction',
                        help='Type of TSR (suction/gripper).')
    parser.add_argument('task', type=str, default='pick',
                        help='Task (pick/stow).')

    args = parser.parse_args(rospy.myargv()[1:])
    tsr_viz = RvizTSRVisualizer()

    tsr_viz.visualizeTSRs(objname=args.objname, tsr_type=args.tsr_type, task=args.task)