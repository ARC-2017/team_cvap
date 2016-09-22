#!/usr/bin/env python

import openravepy
import math
import numpy
import yaml
import tf_conversions

class OpenRAVEUtils:
    def __init__(self):
        self.handles = []
        self.colors = [numpy.array([1, 0, 0, 1]), numpy.array([0, 1, 0, 1]), numpy.array([0, 0, 1, 1])]

    def clearHandles(self):
        self.handles = []

    def drawPose(self, env, transform, length=0.05, width=0.002):
        origin = transform[0:3, 3]
        with env:
            for i in range(0, 3):
                axis = transform[0:3, i]
                self.handles.append(env.drawarrow(origin, origin + length * axis, width, self.colors[i]))

    def updatePoses(self, env, poses):
        self.clearHandles()
        for pose in poses:
            self.drawPose(env, pose)


def computeBinPoses(basePose, columnWidths, rowHeights):
    poses = []
    x = 0.0
    for c in range(0, 3):
        poses.append([])
        y = 0.0
        for r in range(0, 4):
            pose = numpy.array(basePose)
            pose[0, 3] += x
            pose[1, 3] += y
            y += rowHeights[r]
            poses[c].append(pose)
        x += columnWidths[c]
    return poses


def computePosesWithinBin(binPose, binWidth, binHeight, binDepth, rotMin, rotMax, 
                          padding=0.02, numWidthSamples=1, numHeightSamples=1, numDepthSamples=1,
                          numRotSamples=1):
    samplePoses = []
    pose = numpy.array(binPose)
    pose[0, 3] += padding
    pose[1, 3] += padding
    pose[2, 3] -= padding
    rotStep = (rotMax - rotMin) / float(numRotSamples+1)
    for wSample in range(1, numWidthSamples+1):
        pose[0, 3] = binPose[0, 3] + wSample * (binWidth - 2 * padding) / float(numWidthSamples+1) + padding
        for hSample in range(1, numHeightSamples+1):
            pose[1, 3] = binPose[1, 3] + hSample * (binHeight - 2 * padding) / float(numHeightSamples+1) + padding
            for dSample in range(1, numDepthSamples+1):
                pose[2, 3] = binPose[2, 3] - dSample * (binDepth - 2 * padding) / float(numDepthSamples+1) - padding
                for rxSample in range(1, numRotSamples + 1):
                    rxMatrix = numpy.array([[1.0, 0.0, 0.0, 0.0], 
                                            [0.0, math.cos(rxSample * rotStep + rotMin), math.sin(rxSample * rotStep + rotMin), 0.0],
                                            [0.0, -math.sin(rxSample * rotStep + rotMin), math.cos(rxSample * rotStep + rotMin), 0.0],
                                            [0.0, 0.0, 0.0, 1.0]])
                    for rySample in range(1, numRotSamples + 1):
                        ryMatrix = numpy.array([[math.cos(rySample * rotStep + rotMin), 0.0, -math.sin(rySample * rotStep + rotMin), 0.0],
                                                [0.0,                          1.0,                           0.0, 0.0],
                                                [math.sin(rySample * rotStep + rotMin), 0.0,  math.cos(rySample * rotStep + rotMin), 0.0],
                                                [0.0,                          0.0,                           0.0, 1.0]])
                        for rzSample in range(1, numRotSamples + 1):
                            rzMatrix = numpy.array([[ math.cos(rzSample * rotStep + rotMin), math.sin(rzSample * rotStep + rotMin), 0.0, 0.0],
                                                    [-math.sin(rzSample * rotStep + rotMin), math.cos(rzSample * rotStep + rotMin), 0.0, 0.0],
                                                    [                          0.0,                          0.0, 1.0, 0.0],
                                                    [                          0.0,                          0.0, 0.0, 1.0]])
                            samplePoses.append(numpy.array(numpy.dot(pose, numpy.dot(rxMatrix, numpy.dot(ryMatrix, rzMatrix)))))
    return samplePoses


def flatten(listlist):
    flattenedList = []
    for elList in listlist:
        for el in elList:
            flattenedList.append(el)
    return flattenedList

def computeSamplePoses(basePose, rowHeights, columnWidths):
    binPoses = computeBinPoses(basePose, columnWidths, rowHeights)
    allSamplePoses = []
    for c in range(0, 3):
        for r in range(0, 4):
            innerBinPoses = computePosesWithinBin(binPoses[c][r], columnWidths[c], rowHeights[r], 0.05, 0.0, 0.0)
            allSamplePoses.extend(innerBinPoses)
    return allSamplePoses

def posesToYaml(poses, poseNames):
    yamlPoses = {}

    for pId in range(0, len(poses)):
        pose = poses[pId]
        poseEntry = {}
        position = {}
        position['x'] = float(pose[0, 3])
        position['y'] = float(pose[1, 3])
        position['z'] = float(pose[2, 3])
        poseEntry['position'] = position
        orientation = {}
        quaternion = tf_conversions.transformations.quaternion_from_matrix(pose)
        orientation['x'] = float(quaternion[0])
        orientation['y'] = float(quaternion[1])
        orientation['z'] = float(quaternion[2])
        orientation['w'] = float(quaternion[3])
        poseEntry['orientation'] = orientation
        if pId < len(poseNames):
            yamlPoses[poseNames[pId]] = poseEntry
        else:
            yamlPoses[pId] = poseEntry
    return yaml.dump(yamlPoses)


if __name__ == '__main__':
    env = openravepy.Environment()
    env.Load('../data/shelf.kinbody.xml')
    env.SetViewer('qtcoin')
    drawer = OpenRAVEUtils()

    # basePose = numpy.array([[-1, 0, 0, -0.42], [0, 1, 0, 0.84], [0, 0, -1, 0.46], [0, 0, 0, 1]])
    basePose = numpy.array([[-1, 0, 0, -0.42], [0, 1, 0, 0.82], [0, 0, -1, 0.5], [0, 0, 0, 1]])
    columnWidths = [0.275, 0.305, 0.275]
    rowHeights = [0.265, 0.23, 0.23, 0.25]
    poseNames = ['bin_J', 'bin_G', 'bin_D', 'bin_A', 'bin_K', 'bin_H', 'bin_E', 'bin_B',
                 'bin_L', 'bin_I', 'bin_F', 'bin_C']
    allSamplePoses = computeSamplePoses(basePose, rowHeights, columnWidths)
    drawer.updatePoses(env, allSamplePoses)
    yamlString = posesToYaml(allSamplePoses, poseNames)
    print yamlString
    import IPython
    IPython.embed()
