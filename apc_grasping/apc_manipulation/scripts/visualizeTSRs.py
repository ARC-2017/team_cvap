#!/usr/bin/env python

import openravepy
import IPython
import argparse
import sys
import utils.TSRs
import numpy
import yaml
import tf.transformations
import time


def saveTSRs(filePath, tsrList):
    dumpFile = open(filePath, 'w')
    yaml.dump(tsrList, dumpFile)
    dumpFile.close()


def loadTSRs(filePath):
    inputFile = open(filePath, 'r')
    tsrList = yaml.load(inputFile)
    inputFile.close()
    return tsrList


def showPose(robot, config):
    transformMatrix = tf.transformations.euler_matrix(config[3], config[4], config[5], 'szyx')
    transformMatrix[:3, 3] = config[:3]
    robot.SetTransform(transformMatrix)


def showExtremes(robot, tsr, side, flipDim):
    config = numpy.copy(tsr.extents[:, side])
    showPose(robot, config)
    time.sleep(0.5)
    config[flipDim] = tsr.extents[flipDim, (side + 1) % 2]
    showPose(robot, config)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Show TSRs')
    parser.add_argument('--tsrPath', dest='tsrPath', nargs='?', default='',
                        help='Set file to read TSRs from.')
    parser.add_argument('object', help='Path to object to display TSRs next to.')
    args = parser.parse_args()
    # utils.TSRs.addNumpyYaml()
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    robotPath = '../data/openrave/baxter_gripper_kinbody.xml'
    robotLoaded = env.Load(robotPath)
    if not robotLoaded:
        print 'Failed loading robot ' + robotPath
        sys.exit(-1)

    fileLoaded = env.Load(args.object)
    if not fileLoaded:
        print 'Failed loading object ' + fileLoaded
        sys.exit(-1)

    # TODO read in TSR file
    gripper = None
    object = None
    for body in env.GetBodies():
        if body.GetName() == 'baxter_gripper':
            gripper = body
        else:
            object = body

    centerPose = numpy.eye(4, 4)
    object.SetTransform(centerPose)

    gripperPose = numpy.eye(4, 4)
    gripperPose[0, 3] = 0.2
    gripperPose[1, 3] = 0.2
    gripper.SetTransform(gripperPose)

    tsr = utils.TSRs.BoxTSR()
    IPython.embed()
