#!/usr/bin/env python
"""
    This script contains a ROS node that computes a roadmap for Baxter for the APC.

    @author: Joshua Haustein (haustein@kth.se)
"""

# ROS/Baxter/MoveIt
import rospy
import rospkg
import tf
import moveit_commander
import baxter_interface
import actionlib
from moveit_msgs.msg import MoveGroupAction, RobotState
from geometry_msgs.msg import PoseStamped
# apc imports
import utils.ArgumentsCollector
import utils.roadmap
from apc_linear_path_planner.srv import InvertTrajectory
# python imports
import sys
import yaml
import pickle
import IPython


class RoadmapCreator:

    def __init__(self):
        self._robotCommander = moveit_commander.RobotCommander()
        self._moveitScene = moveit_commander.PlanningSceneInterface()
        self._leftArmGroup = moveit_commander.MoveGroupCommander('left_arm')
        self._rightArmGroup = moveit_commander.MoveGroupCommander('right_arm')
        self._trajectoryInverter = rospy.ServiceProxy('/apc/invert_trajectory', InvertTrajectory)
        self._tfListener = tf.TransformListener()
        self._head = baxter_interface.Head()
        self._collisionObjects = []
        self._collisionMeshesRootDir = ""
        self._namedConfigurations = {}
        self._rightJointNames = ['right_s0', 'right_s1', 'right_e0', 'right_e1',
                                 'right_w0', 'right_w1', 'right_w2']
        self._leftJointNames = ['left_s0', 'left_s1', 'left_e0', 'left_e1',
                                'left_w0', 'left_w1', 'left_w2']

    def setConfiguration(self, configFileName, filePath, prefix):
        self._outputFilePath = filePath
        self._outputFilePrefix = prefix
        if configFileName is not None:
            try:
                configFile = open(configFileName, 'r')
                self._namedConfigurations = yaml.load(configFile)
                configFile.close()
            except IOError:
                rospy.logerr('Could not load configurations from file ' + configFileName)

    def setupPlanningScene(self, roadmapType):
        success = False
        try:
            self._collisionObjects = rospy.get_param('/apc/manipulation/collision_objects')
        except:
            rospy.logerr('Could not initialize, missing collision_objects param')
            success = False
            return success

        try:
            self._collisionMeshesRootDir = rospkg.RosPack().get_path('apc_2016_mesh_models') + '/object_models/'
        except:
            rospy.logerr('Could not find path to apc_2016_mesh_models package')
            success = False
            return success

        while not rospy.is_shutdown() and not success:
            for cObject in self._collisionObjects:
                hasTransform = False
                hasMesh = False
                while not hasTransform and not rospy.is_shutdown():
                    try:
                        (trans, rot) = self._tfListener.lookupTransform('/base', cObject['name'], rospy.Time(0))
                        hasTransform = True
                        cObject['pose'] = utils.ArgumentsCollector.createROSPose(trans, rot)

                    except:
                        rospy.logwarn('Could not receive ' + cObject['name'] + ' pose.')
                        hasTransform = False
                        rospy.sleep(1.0)

                while not hasMesh and not rospy.is_shutdown():
                    try:
                        self._moveitScene.add_mesh(
                            # pose of ROSPose type
                            cObject['name'], cObject['pose'], self._collisionMeshesRootDir + cObject['mesh_path'])
                        hasMesh = True
                        success = True
                    except:
                        errorMsg = 'Mesh stored at %s  is invalid. Please fix this!' % (self._collisionMeshesRootDir + cObject['mesh_path'])
                        rospy.logerr(errorMsg)
                        hasMesh = False
                        rospy.sleep(1.0)
            success = True

        # rospy.sleep(2.0)
        poseStamped = PoseStamped()
        touchLinks = ['left_gripper_base', 'left_gripper_extension',
                      'left_hand', 'left_hand_camera', 'left_hand_accelerometer',
                      'left_hand_range', 'left_wrist']
        if roadmapType == 'picking':
            objectDimensions = [0.1, 0.1, 0.1]
            poseStamped.header.frame_id = 'left_gripper'
            poseStamped.pose.position.z = 0.03
        else:
            objectDimensions = [0.1, 0.1, 0.1]
            poseStamped.header.frame_id = 'left_gripper_base'
            poseStamped.pose.position.z = 0.1
        self._moveitScene.attach_box('left_gripper', 'object_bounding_box_left', poseStamped, objectDimensions, touchLinks)

        rospy.sleep(1.0)
        objectDimensions = [0.1, 0.1, 0.1]
        poseStamped.header.frame_id = 'right_gripper'
        poseStamped.pose.position.z = 0.03
        touchLinks = ['right_gripper_base', 'right_gripper_extension',
                      'right_hand', 'right_hand_camera', 'right_hand_accelerometer',
                      'right_hand_range', 'right_wrist']
        self._moveitScene.attach_box('right_gripper', 'object_bounding_box_right', poseStamped, objectDimensions, touchLinks)

        return success

    def computeTraj(self, moveGroup, start, goal, timeout=8.0):
        startState = RobotState()
        keyValuePairs = start.items()
        startState.joint_state.name = map(lambda x: x[0], keyValuePairs)
        startState.joint_state.position = map(lambda x: x[1], keyValuePairs)
        try:
            moveGroup.set_start_state(startState)
            moveGroup.set_joint_value_target(goal)
            moveGroup.set_planning_time(timeout)
            moveGroup.set_planner_id('RRTStarkConfigDefault')
            moveit_traj = moveGroup.plan()
            return (len(moveit_traj.joint_trajectory.points) > 0, moveit_traj)
        except moveit_commander.MoveItCommanderException as err:
            rospy.logwarn('MoveIt planning failed: ' + str(err))
            return (False, None)

    def moveHeadAside(self, arm):
        pan = 1.5
        if arm == 'left_arm':
            pan = -1.5
        self._head.set_pan(pan)

    def computeRoadmap(self, arm):
        moveGroup = self._leftArmGroup
        roadmap = utils.roadmap.Roadmap()
        if arm == 'right_arm':
            moveGroup = self._rightArmGroup
        # first move both arms home
        success = self._moveArmHome('left_arm')
        if not success:
            return None
        success = self._moveArmHome('right_arm')
        if not success:
            return None
        # Move head aside
        self.moveHeadAside(arm)

        # now run through all named configurations and create a node for each
        namedConfigs = self._namedConfigurations[arm].items()
        allNodes = []
        for i in range(len(namedConfigs)):
            node = utils.roadmap.RoadmapNode(config=namedConfigs[i][1],
                                             name=namedConfigs[i][0])
            roadmap.addNode(node)
            allNodes.append(node)

        # now compute the edges
        numTotalEdges = len(allNodes) * (len(allNodes) - 1)
        numFinishedEdgeChecks = 0
        numAddedEdges = 0
        rospy.loginfo('We have ' + str(len(allNodes)) + ' nodes. Hence, we need to check ' +
                      str(numTotalEdges) + ' edges.')
        for i in range(len(allNodes)):
            for j in range(i + 1, len(allNodes)):
                (success, traj) = self.computeTraj(moveGroup, start=allNodes[i].getConfig(),
                                                   goal=allNodes[j].getConfig())
                numFinishedEdgeChecks += 2
                if success:
                    allNodes[i].addEdge(allNodes[j], traj)
                    invTraj = self._invertTrajectory(traj=traj, mgName=moveGroup.get_name())
                    allNodes[j].addEdge(allNodes[i], invTraj)
                    numAddedEdges += 2
                else:
                    rospy.loginfo('Could not find a trajectory between ' + allNodes[i].getName() +
                                  ' and ' + allNodes[j].getName())
                rospy.loginfo('Checked ' + str(numFinishedEdgeChecks) + ' / ' + str(numTotalEdges) +
                              ' so far. The roadmap has currently ' + str(numAddedEdges) + ' edges')
                if rospy.is_shutdown():
                    break

        # Check whether the roadmap is strongly connected
        stronglyConnected = roadmap.isStronglyConnected()
        if not stronglyConnected:
            rospy.logwarn('The computed roadmap is not strongly connected.')
        else:
            rospy.loginfo('All good. The computed roadmap is strongly connected.')
        # once we are done with this, save the results
        fileName = self._outputFilePath + '/' + self._outputFilePrefix + '_' + arm + '.pickle'
        rospy.loginfo('Roadmap computation finished. Saving roadmap in ' + fileName)
        try:
            afile = open(fileName, 'w')
            pickle.dump(roadmap, afile)
            afile.close()
        except Exception as err:
            rospy.logerr('Could not save roadmap in file ' + fileName + ' because: ' + str(err))
        return roadmap

    def _invertTrajectory(self, traj, mgName):
        result = self._trajectoryInverter(traj=traj, move_group=mgName)
        return result.traj

    def _moveArmHome(self, arm):
        jointNames = self._leftJointNames
        moveGroup = self._leftArmGroup
        if arm == 'right_arm':
            jointNames = self._rightJointNames
            moveGroup = self._rightArmGroup
        jointValues = moveGroup.get_current_joint_values()
        config = {}
        for i in range(len(jointValues)):
            config[jointNames[i]] = jointValues[i]

        for i in range(10):
            (success, traj) = self.computeTraj(moveGroup, start=config,
                                               goal=self._namedConfigurations[arm]['HOME'],
                                               timeout=8.0)
            if success:
                break
        if not success:
            rospy.logerr('Could not compute roadmap. Planning to HOME for ' + arm + ' failed')
            return False
        else:
            moveGroup.execute(traj)
            return True

if __name__ == '__main__':
    rospy.init_node('apc_compute_roadmap_node')
    # wait for move_group action to become available to avoid timeout
    move_group_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
    rospy.loginfo('[apc_grasping/MoveItInterface]: waiting for move_group action server')
    move_group_client.wait_for_server()
    rospy.wait_for_service('/apc/invert_trajectory')
    moveit_commander.roscpp_initialize(sys.argv)
    pathToConfigFile = rospy.get_param('/apc/manipulation/named_configs')
    # pathToConfigFileStowing = rospy.get_param('/apc/manipulation/named_configs_stowing')

    outputFilePath = rospy.get_param('/apc/manipulation/roadmap_path', './')
    outputFilePrefix = rospy.get_param('/apc/manipulation/roadmap_name', 'apc_roadmap')
    roadmapType = rospy.get_param('/apc/manipulation/roadmap_type', 'picking')
    rc = RoadmapCreator()
    rc.setupPlanningScene(roadmapType)
    allGood = True
    IPython.embed()
    if allGood:
        rc.setConfiguration(configFileName=pathToConfigFile, filePath=outputFilePath, prefix=outputFilePrefix)
        leftRoadmap = rc.computeRoadmap('left_arm')
        rightRoadmap = rc.computeRoadmap('right_arm')
        rospy.loginfo('Roadmap computation is finished.')
        if leftRoadmap is not None:
            rospy.loginfo('Strongly connected? left: ' + str(leftRoadmap.isStronglyConnected()))
        else:
            rospy.logerr('Failed to compute a roadmap for the left arm')
        if rightRoadmap is not None:
            rospy.loginfo('Strongly connected? right: ' + str(rightRoadmap.isStronglyConnected()))
        else:
            rospy.logerr('Failed to compute a roadmap for the right arm')

    IPython.embed()
    rospy.spin()
