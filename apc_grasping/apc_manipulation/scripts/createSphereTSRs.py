#!/usr/bin/env python
"""
  Computes TSRs for suction for spheres.
"""

import IPython
import yaml
import utils.TSRs
import argparse
import numpy
import PyKDL as kdl
import tf.transformations



def kdlFrameToBoxTSR(kdl_frame):

    tsr = utils.TSRs.BoxTSR()
    for i in range(3):
        tsr.extents[i] = [kdl_frame.p[i], kdl_frame.p[i]]

    zyx_rot = tf.transformations.euler_from_quaternion(kdl_frame.M.GetQuaternion(), 'szyx')

    for i in range(3):
        tsr.extents[i+3] = [zyx_rot[i], zyx_rot[i]]

    return tsr



def computeTSRsForSphere(radius, angleStep=30*numpy.pi/180.0):

    radial_vector = kdl.Vector(0.0, 0.0, 1.0) * radius

    x_angle_grid = numpy.linspace(0.0, 2*numpy.pi, int(2*numpy.pi/angleStep))
    y_angle_grid = numpy.copy(x_angle_grid)

    tsrs = []

    for x_angle in x_angle_grid:
        for y_angle in y_angle_grid:
            R = kdl.Rotation.RPY(x_angle, y_angle, 0.0)
            pos = R * radial_vector

            rot = R * kdl.Rotation.RotX(numpy.pi)

            tsrs.append(kdlFrameToBoxTSR(kdl.Frame(rot, pos)))

    return tsrs

if __name__ == "__main__":
    # utils.TSRs.addNumpyYaml()
    parser = argparse.ArgumentParser(description='Compute TSRs for a sphere!')
    parser.add_argument('radius',type=float,
                        help='The radius of the sphere in meters.')
    parser.add_argument('--output', dest='outputPath', nargs='?', default='',
                        help='Path to file to store TSRs in.')
    # TODO add transform of object frame to centered frame.
    # parser.add_argument('object', help='Path to object to display TSRs next to.')
    args = parser.parse_args()
    tsrs = computeTSRsForSphere(radius = args.radius)
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
