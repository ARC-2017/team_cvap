#!/usr/bin/env python
"""
  Computes TSRs for suction for cylinders.
"""

import IPython
import yaml
import utils.TSRs
import argparse
import numpy
import PyKDL as kdl
import tf.transformations


def computeTSRsForCylinderLid(innerRadius, outerRadius, height, mainAxis, suctionRadius, padding=5e-3, radialStep=2e-3):
    tsrs = []

    if innerRadius == 0.0:
        usable_radial_dist = (outerRadius - innerRadius) - (padding + suctionRadius)

    else:
        usable_radial_dist = (outerRadius - innerRadius) - 2*(padding + suctionRadius)

    if usable_radial_dist <= 0.0:
        return tsrs

    if innerRadius == 0.0:
        radial_grid = numpy.linspace(innerRadius,
                                     outerRadius - (padding + suctionRadius),
                                     max(usable_radial_dist/radialStep, 1))
    else:
        radial_grid = numpy.linspace(innerRadius + padding + suctionRadius,
                                     outerRadius - (padding + suctionRadius),
                                     max(usable_radial_dist/radialStep, 1))

    main_axis = getKDLAxis(mainAxis)

    angle_sign = height/abs(height)

    for radius in radial_grid:
        frames = computeFramesOnCircle(radius, mainAxis)
        for frame in frames:
            if mainAxis == 'x':
                frame.M = kdl.Rotation.RotY(-angle_sign*numpy.pi*0.5)
            elif mainAxis == 'y':
                frame.M = kdl.Rotation.RotX(angle_sign*numpy.pi*0.5)
            else:
                if angle_sign > 0:
                    frame.M = kdl.Rotation.RotX(numpy.pi)
                else:
                    frame.M = kdl.Rotation()

            frame.p = frame.p + height*main_axis

            tsrs.append(kdlFrameToBoxTSR(frame))


    return tsrs

def kdlFrameToBoxTSR(kdl_frame):

    tsr = utils.TSRs.BoxTSR()
    for i in range(3):
        tsr.extents[i] = [kdl_frame.p[i], kdl_frame.p[i]]

    zyx_rot = tf.transformations.euler_from_quaternion(kdl_frame.M.GetQuaternion(), 'szyx')

    for i in range(3):
        tsr.extents[i+3] = [zyx_rot[i], zyx_rot[i]]

    return tsr

def getKDLAxis(mainAxis):
    '''

    @param mainAxis: (str) 'x', 'y' or 'z'
    @return:
    '''

    if mainAxis=='x':
        main_axis = kdl.Vector(1, 0, 0)
    elif mainAxis=='y':
        main_axis = kdl.Vector(0, 1, 0)
    else:
        main_axis = kdl.Vector(0, 0, 1)

    return main_axis

def computeFramesOnCircle(radius, mainAxis, thetaStep=10*numpy.pi/180.0):
    '''
    Computes poses on a circle, with orientation such that the z-axis points
    towards the center of the circle.
    @param radius:
    @param mainAxis:
    @param thetaStep:
    @return:
    '''

    theta_grid = numpy.linspace(0, 2*numpy.pi, 2*numpy.pi/thetaStep)

    # normal vector from center to edge of circle
    # tsr_rot makes Z axis point towards center of circle
    if mainAxis=='x':
        normal_vector = kdl.Vector(0, 0, 1)
        tsr_rot = kdl.Rotation.RotX(numpy.pi)
    elif mainAxis=='y':
        normal_vector = kdl.Vector(0, 0, 1)
        tsr_rot = kdl.Rotation.RotY(numpy.pi)
    else:
        normal_vector = kdl.Vector(1, 0, 0)
        tsr_rot = kdl.Rotation.RPY(numpy.pi*0.5, 0.0, -numpy.pi*0.5)

    frames = []
    for theta in theta_grid:
        if mainAxis=='x':
            cyl_rot = kdl.Rotation.RotX(theta)
        elif mainAxis=='y':
            cyl_rot = kdl.Rotation.RotY(theta)
        else:
            cyl_rot = kdl.Rotation.RotZ(theta)

        frames.append(kdl.Frame())
        frames[-1].p = cyl_rot * normal_vector * radius
        frames[-1].M = cyl_rot * tsr_rot

    return frames

def computeTSRsForCylinderFace(radius, height, mainAxis, suctionRadius, padding=5e-3, height_step=5e-3):

    tsrs = []
    usable_height = height - 2*(padding + suctionRadius)

    if usable_height <= 0.0:
        return tsrs

    height_grid = numpy.linspace(-height*0.5 + suctionRadius + padding,
                                 height*0.5 - (suctionRadius + padding),
                                 max(usable_height/height_step, 1))

    circle_frames = computeFramesOnCircle(radius, mainAxis)

    main_axis = getKDLAxis(mainAxis)

    for h in height_grid:
        for frame in circle_frames:

            tsrs.append(kdlFrameToBoxTSR(kdl.Frame(frame.M, frame.p + main_axis*h)))

    return tsrs

def computeTSRsForCylinder(innerRadius, outerRadius, height, mainAxis, lidTSRs, suctionRadius):

    tsr_cylinder_face = []
    tsr_top_lid = []
    tsr_bottom_lid = []

    tsr_cylinder_face = computeTSRsForCylinderFace(radius = outerRadius,
                                           height = height,
                                           mainAxis = mainAxis,
                                           suctionRadius = suctionRadius)

    if lidTSRs:
        tsr_top_lid = computeTSRsForCylinderLid(innerRadius = innerRadius,
                                              outerRadius = outerRadius,
                                              height = height*0.5,
                                              suctionRadius = suctionRadius,
                                              mainAxis = mainAxis)

        tsr_bottom_lid = computeTSRsForCylinderLid(innerRadius = innerRadius,
                                              outerRadius = outerRadius,
                                              height = -height*0.5,
                                              suctionRadius = suctionRadius,
                                              mainAxis = mainAxis)

    tsrs = tsr_cylinder_face + tsr_top_lid + tsr_bottom_lid

    return tsrs


if __name__ == "__main__":
    # utils.TSRs.addNumpyYaml()
    parser = argparse.ArgumentParser(description='Compute TSRs for a cylinder!')
    parser.add_argument('innerRadius',type=float,
                        help='The inner radius of the cylinder in meters.')
    parser.add_argument('outerRadius', type=float,
                        help='The outer radius of the cylinder in meters.')
    parser.add_argument('height', type=float,
                        help='The height of the cylinder in meters.')
    parser.add_argument('mainAxis', type=str,
                        help='Main axis of the cylinder  (x, y or z)')
    parser.add_argument('lidTSRs', type=int,
                        help='Calculate TSRs for lids of cylinder or not (1 / 0)')
    parser.add_argument('suctionRadius', type=float,
                        help='The radius of the suction cup in meters.')
    parser.add_argument('--output', dest='outputPath', nargs='?', default='',
                        help='Path to file to store TSRs in.')
    # TODO add transform of object frame to centered frame.
    # parser.add_argument('object', help='Path to object to display TSRs next to.')
    args = parser.parse_args()
    tsrs = computeTSRsForCylinder(innerRadius=args.innerRadius,
                                  outerRadius=args.outerRadius,
                                  height=args.height,
                                  mainAxis=args.mainAxis,
                                  lidTSRs=args.lidTSRs,
                                  suctionRadius=args.suctionRadius)
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
