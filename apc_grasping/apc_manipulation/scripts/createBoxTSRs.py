#!/usr/bin/env python
"""
  Computes TSRs for suction for boxes.
"""

import IPython
import yaml
import utils.TSRs
import argparse
import numpy

def setRotationRanges(face, tsr):
    xAxis = numpy.array([1,0,0])
    yAxis = numpy.array([0,1,0])
    zAxis = numpy.array([0,0,1])
    normal = 1/numpy.linalg.norm(face[2]) * face[2]
    if numpy.dot(normal, zAxis) >= 0.99:  # normal == zAxis
        tsr.extents[3] = [-3.14, 3.14]
        tsr.extents[4] = [3.14, 3.15]
        tsr.extents[5] = [-0.001, 0.001]
    elif numpy.dot(normal, -zAxis) >= 0.99:  # normal == -zAxis
        tsr.extents[3] = [-3.14, 3.14]
        tsr.extents[4] = [-0.001, 0.001]
        tsr.extents[5] = [-0.001, 0.001]
    elif numpy.dot(normal, xAxis) >= 0.99:  # normal == xAxis
        tsr.extents[3] = [-0.001, 0.001]
        tsr.extents[4] = [-1.55, -1.53]
        tsr.extents[5] = [-3.14, 3.14]
    elif numpy.dot(normal, -xAxis) >= 0.99:  # normal == -xAxis
        tsr.extents[3] = [-0.001, 0.001]
        tsr.extents[4] = [1.53, 1.55]
        tsr.extents[5] = [-3.14, 3.14]
    elif numpy.dot(normal, yAxis) >= 0.99:  # normal == yAxis
        tsr.extents[3] = [-3.14, 3.14]
        tsr.extents[4] = [-0.001, 0.001]
        tsr.extents[5] = [1.56, 1.58]
    elif numpy.dot(normal, -yAxis) >= 0.99:  # normal == yAxis
        tsr.extents[3] = [-3.14, 3.14]
        tsr.extents[4] = [-0.001, 0.001]
        tsr.extents[5] = [-1.58, -1.56]


def computeTSRForFace(face, suctionRadius):
    width = numpy.linalg.norm(face[0])
    height = numpy.linalg.norm(face[1])
    wDir = 1.0 / width * face[0]
    hDir = 1.0 / height * face[1]
    wAvailable = width - 2.0 * suctionRadius
    hAvailable = height - 2.0 * suctionRadius
    if wAvailable <= 0.0 or hAvailable <= 0.0:
        return None
    tsr = utils.TSRs.BoxTSR()
    setRotationRanges(face, tsr)
    centerPoint = 0.5 * face[2]
    minPoint = centerPoint - wAvailable/2.0 * wDir - hAvailable/2.0 * hDir
    maxPoint = centerPoint + wAvailable/2.0 * wDir + hAvailable/2.0 * hDir
    for i in range(3):
        tsr.extents[i] = [float(minPoint[i]), float(maxPoint[i])]
    return tsr


def computeTSRsForBox(width, height, depth, suctionRadius):
    tsrs = []
    # Each face is represented as a triple (w * axis1, h * axis2, d * normal)
    faces = [(numpy.array([width, 0, 0]), numpy.array([0, height, 0]), numpy.array([0, 0, depth])),
             (numpy.array([width, 0, 0]), numpy.array([0, height, 0]), numpy.array([0, 0, -depth])),
             (numpy.array([0, 0, depth]), numpy.array([0, height, 0]), numpy.array([width, 0, 0])),
             (numpy.array([0, 0, depth]), numpy.array([0, height, 0]), numpy.array([-width, 0, 0])),
             (numpy.array([0, 0, depth]), numpy.array([width, 0, 0]), numpy.array([0, height, 0])),
             (numpy.array([0, 0, depth]), numpy.array([width, 0, 0]), numpy.array([0, -height, 0]))]
    for face in faces:
        tsr = computeTSRForFace(face, suctionRadius)
        if tsr is not None:
            tsrs.append(tsr)
    return tsrs


if __name__ == "__main__":
    # utils.TSRs.addNumpyYaml()
    parser = argparse.ArgumentParser(description='Compute TSRs for a box!')
    parser.add_argument('width',type=float,
                        help='The width of the box in meters.')
    parser.add_argument('height', type=float,
                        help='The height of the box in meters.')
    parser.add_argument('depth', type=float,
                        help='The depth of the box in meters.')
    parser.add_argument('suctionRadius', type=float,
                        help='The radius of the suction cup in meters.')
    parser.add_argument('--output', dest='outputPath', nargs='?', default='',
                        help='Path to file to store TSRs in.')
    # TODO add transform of object frame to centered frame.
    # parser.add_argument('object', help='Path to object to display TSRs next to.')
    args = parser.parse_args()
    tsrs = computeTSRsForBox(args.width, args.height, args.depth, args.suctionRadius)
    if not args.outputPath == '':
        try:
            f = open(args.outputPath, 'w')
            yaml.dump(tsrs, f)
            f.close()
        except IOError as e:
            print 'Could not write TSRs to file. Reason: \n' + str(e)
            print 'The TSRs are: \n', tsrs
    else:
        print 'The TSRs are: \n', tsrs
    # IPython.embed()
