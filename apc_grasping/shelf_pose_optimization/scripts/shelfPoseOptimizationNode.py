#!/usr/bin/env python

import rospy
import sys
import argparse
sys.path.append('/home/joshua/projects/apc_grasping/devel/lib/')
import libshelf_pose_optimization
import pysmac
import time
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

if __name__ == "__main__":
    # rospy.init_node('shelf_optimization_node')
    roscpp_init('shelf_pose_optimization_node', [])
    parser = argparse.ArgumentParser(description='Script to compute an optimal shelf pose in respect to baxter.')
    parser.add_argument('--shelfMeshPath', nargs='?', default='../data/pod_lowres.stl',
                        help='Path to shelf mesh.')
    argv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(args=argv[1:])

    evaluatorArgs = libshelf_pose_optimization.Arguments()
    evaluatorArgs.paramName = "robot_description"
    evaluatorArgs.paramsFromServer = True
    evaluatorArgs.shelfMeshPath = args.shelfMeshPath
    evaluatorArgs.numEEFRotSamples = 2
    evaluatorArgs.numEEFWidthSamples = 3
    evaluatorArgs.numEEFHeightSamples = 3
    evaluatorArgs.numEEFDepthSamples = 3

    evaluator = libshelf_pose_optimization.ShelfPoseOptimizer()
    evaluator.setScenePublisher("/shelf_optimization_scene")
    evaluator.setPosesPublisher("/shelf_bin_sample_poses")
    evaluator.initialize(evaluatorArgs)

    def evaluatePose(x, y, phi):
        poseQuality = evaluator.evaluateShelfPose(x, y, phi, False, False)
        return -1.0 * poseQuality

    def evaluatePoseFixed(x):
        poseQuality = evaluator.evaluateShelfPose(x, 0.0, 0.0, False, False)
        return -1.0 * poseQuality

    def bruteForceOptimization(xmin, xmax, stepSize=0.01):
        x = xmin
        bestX = xmin
        bestScore = 0.0
        while x < xmax:
            score = evaluatePoseFixed(x)
            print 'Score at %f is %f' %(x, score)
            if score < bestScore:
                bestScore = score
                bestX = x
            x += stepSize
        return bestScore, bestX

    # parameter_def = dict(x=('real', [0.9, 1.5], 1.2),
    #                      y=('real', [-0.5, 0.5], 0.0),
    #                      phi=('real', [-0.3, 0.3], 0.0))
    # parameter_def = dict(x=('real', [0.8, 1.6], 1.2))
    #                      y=('real', [-0.5, 0.5], 0.0),
    #                      phi=('real', [-0.3, 0.3], 0.0))
    # optimizer = pysmac.SMAC_optimizer()
    # value, bestPose = optimizer.minimize(evaluatePoseFixed, 50, parameter_def)
    startTime = time.time()
    value, bestPose = bruteForceOptimization(1.15, 1.25)
    totalTime = time.time() - startTime

    print "The best pose is ", bestPose
    print "Its value is ", -value
    print "Optimization took %f s" % totalTime
    import IPython
    IPython.embed()

# The best pose is  {'y': '0.16336234381413184', 'x': '1.183508394192143', 'phi': '0.1883172065000388'}
# Its value is  0.32876086235

# x=1.2313976405566547