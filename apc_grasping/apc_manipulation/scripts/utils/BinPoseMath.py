#!/usr/bin/env python
import PyKDL as kdl
import numpy
import ArgumentsCollector
from tf_conversions import posemath


def sampleOptimalZAngle(moveItInterface, R_bin_sample, R_eef_gripper_base, desiredZAxis, numAngles):
    '''
    Find optimal angle for rotating a pose sample around Z-axis so that the gripper base's
    z axis aligns as much as possible with the desired z axis w.r.t the bin.
    @param R_bin_sample: rotation of the sample w.r.t. bin frame (kdl.Rotation)
    @param R_eef_gripper_base: rotation of the eef (tool-tip) w.r.t. gripper base frame (kdl.Rotation)
    @return: optimal angle (float)
    '''
    z_bin_gripper_base_desired = kdl.Vector(*(desiredZAxis))

    # evaluate for a series of rotations around z axis
    thetas_z = numpy.linspace(0, 2 * numpy.pi, numAngles)
    dot_products = numpy.array([])

    for angle in thetas_z:
        moveItInterface.checkPreemption()
        R_bin_sample_rotated = R_bin_sample * kdl.Rotation.RotZ(angle)

        # resulting Z-axis of gripper_base
        z_bin_gripper_base = R_bin_sample_rotated * R_eef_gripper_base * kdl.Vector(0.0, 0.0, 1.0)

        dot_products = numpy.append(dot_products, kdl.dot(z_bin_gripper_base, z_bin_gripper_base_desired))

    optimal_angle = thetas_z[dot_products.argmax()]

    return optimal_angle


def optimizeOrientationSamples(moveItInterface, binName, eefPoseInGripperBase, goalFrameName,
                               poseSamples, noiseStdev, desiredZAxis, numAngles):
    """
    Optimizes orientation around Z-axis of pose samples as to achieve configurations for the robot arm
    that are less likely to be in collision with the shelf.
    We try to optimize an equation of the form Rz(theta) * x = y
    @param eefPoseInGripperBase: current pose of the end effector w.r.t. gripper base frame (PoseStamped)
    @param goalFrameName: name of the goal frame (e.g. object frame name)
    @param poseSamples: list of poses (list of PoseStamped)
    @param desiredZAxis: the desired zaxis of the gripper base frame w.r.t. base frame
    @param numAngles: the number of angles to try for the pose optimization (the more the merrier, but slower)
    @return: pose samples with optimized orientation
    """
    binPose = moveItInterface.getTransform(binName)
    F_base_bin = posemath.fromMsg(binPose.pose)
    # F_bin_base = F_base_bin.Inverse()

    F_gripper_base_eef = posemath.fromMsg(eefPoseInGripperBase.pose)
    R_eef_gripper_base = F_gripper_base_eef.M.Inverse()

    F_base_goal = posemath.fromMsg((moveItInterface.getTransform(goalFrameName)).pose)

    optimizedSamples = []
    for sample in poseSamples:
        moveItInterface.checkPreemption()
        # transform samples to bin frame
        F_bin_sample = posemath.fromMsg(moveItInterface.transformPose(sample, binName).pose)
        R_bin_sample = F_bin_sample.M

        # get the optimal angle around Z axis
        optimal_angle = sampleOptimalZAngle(moveItInterface, R_bin_sample, R_eef_gripper_base,
                                            desiredZAxis, numAngles)

        # calculate rotated TSR with some gaussian noise and convert to object frame
        noise = numpy.random.normal(0.0, noiseStdev)
        R_bin_sample_rotated = R_bin_sample * kdl.Rotation.RotZ(optimal_angle + noise)
        F_bin_sample_rotated = kdl.Frame(R_bin_sample_rotated, F_bin_sample.p)
        F_obj_sample_rotated = F_base_goal.Inverse() * F_base_bin * F_bin_sample_rotated

        sample.pose = posemath.toMsg(F_obj_sample_rotated)
        sample.header.frame_id = goalFrameName
        optimizedSamples.append(sample)

    return optimizedSamples


def getStowingEEFPose(moveItInterface, binName, arm, desiredGripperBasePose):
    """ Computes the EEF pose for the given arm achieve the desired gripper base pose
        in respect to the frame of bin with name binName.
    """
    gripperFrameName = moveItInterface.getGripperFrameName(arm)    
    gripperBaseFrameName = moveItInterface.getGripperBaseFrameName(arm)
    eefPoseInGripperBase = moveItInterface.getTransform(gripperFrameName, gripperBaseFrameName)
    binPose = moveItInterface.getTransform(binName)

    F_gripper_base_eef = posemath.fromMsg(eefPoseInGripperBase.pose)
    F_base_bin = posemath.fromMsg(binPose.pose)
    F_bin_gripper_base = posemath.fromMsg(desiredGripperBasePose)

    F_base_eef = F_base_bin * F_bin_gripper_base * F_gripper_base_eef
    pose = posemath.toMsg(F_base_eef)
    stampedPose = ArgumentsCollector.makeStamped(pose, 'base')
    return stampedPose
