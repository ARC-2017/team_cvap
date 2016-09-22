#!/usr/bin/env python
""" This file contains a customized path planner for the
    Amazon Picking Challenge. """

# Baxter
import baxter_pykdl
import baxter_interface
# ROS
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from moveit_commander.exception import MoveItCommanderException
# APC
import utils.ArgumentsCollector
import roadmap
from apc_linear_path_planner.srv import PlanLinearPath, InvertTrajectory, ComputeIKSolution, PlanCartesianLinearPath
# Others
import yaml
import pickle
import os
import math
from rtree import index

CONFIGURATION_EPSILON = 0.01
TRAJ_EPSILON = 0.2


def configurationsAreIdentical(configA, configB):
    return roadmap.computeDistanceConfigs(configA, configB) < CONFIGURATION_EPSILON


def isTrajectoryTrivial(traj):
    firstPoint = traj.joint_trajectory.points[0]
    lastPoint = traj.joint_trajectory.points[-1]
    dist = 0.0
    for j in range(len(lastPoint.positions)):
        dist = max(dist, abs(firstPoint.positions[j] - lastPoint.positions[j]))
    return dist <= TRAJ_EPSILON


class APCGlobalPathPlanner():

    def __init__(self, armName, moveGroup, preemptionCheck):
        # wait for linear path planner
        rospy.wait_for_service('/apc/plan_linear_path')
        rospy.wait_for_service('/apc/invert_trajectory')
        rospy.wait_for_service('/apc/compute_ik_solution')
        rospy.wait_for_service('/apc/plan_cartesian_linear_path')
        self._linearPathPlanner = rospy.ServiceProxy('/apc/plan_linear_path', PlanLinearPath)
        self._trajectoryInverter = rospy.ServiceProxy('/apc/invert_trajectory', InvertTrajectory)
        self._inverseKinematicsSolver = rospy.ServiceProxy('/apc/compute_ik_solution', ComputeIKSolution)
        self._cartesianLinearPathPlanner = rospy.ServiceProxy('/apc/plan_cartesian_linear_path',
                                                              PlanCartesianLinearPath)
        self._armName = armName
        self._moveGroup = moveGroup
        self._roadmap = roadmap.Roadmap()
        self._preemptionCheck = preemptionCheck
        # We are using an RTree to save a mapping between poses and configurations in the roadmap
        prop = index.Property()
        prop.dimension = 6
        self._poseIKCache = index.Index(properties=prop)
        jointNames = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']
        if armName == 'left_arm':
            self._armKinematics = baxter_pykdl.baxter_kinematics('left')
            self._limb = baxter_interface.Limb('left')
            self._jointNames = map(lambda x: 'left_' + x, jointNames)
        else:
            self._armKinematics = baxter_pykdl.baxter_kinematics('right')
            self._limb = baxter_interface.Limb('right')
            self._jointNames = map(lambda x: 'right_' + x, jointNames)

    def getRoadmapNodeFromConfig(self, config, numConnectNeighbors=5, allowMoveIt=True):
        """ Searches for the closest node in the roadmap. If this node is
            very close to the given configuration, the node is returned.
            Otherwise it attempts to connect the given configuration to one of the
            numConnectNeighbors closest neighbors in the roadmap
            @param config - configuration for which we want a roadmap node
            @param numConnectNeighbors - number of nearest neighbors to attempt a connection to,
                                    if config is not very close to any roadmap node.
            @return a roadmap node from which we can plan a roadmap path or None if nothing worked.
        """
        self._preemptionCheck()
        nearestNeighbors = self._roadmap.getNearestNeighbors(config, nNeighbors=numConnectNeighbors)
        if len(nearestNeighbors) == 0:
            raise ValueError('No nearest neighbor found for configuration ' + str(config) +
                             '. The roadmap seems to be empty!')
        # let's first check whether we are at a configuration that is already in the roadmap
        for nn in nearestNeighbors:
            rospy.loginfo('Nearest neighbor: ' + nn.getName())

        nn = nearestNeighbors[0]
        if roadmap.computeDistanceConfigs(config, nn.getConfig()) < CONFIGURATION_EPSILON:
            return nn
        # if we reach this point, then we need to connect config to any of our nearest neighbors
        # first try doing this by linear interpolation
        startNode = roadmap.RoadmapNode(config=config, name=self._roadmap.getNewNodeName())
        (connecting_traj, idx) = self._planPathToAny(start=startNode.getConfig(),
                                                     goals=map(lambda x: x.getConfig(), nearestNeighbors),
                                                     allowMoveIt=allowMoveIt)
        if connecting_traj is not None:
            if idx not in range(len(nearestNeighbors)):
                raise ValueError('Received a non None trajectory, but not a valid index for the goal config.')
            endNode = nearestNeighbors[idx]
            startNode.addEdge(endNode, connecting_traj)
            self._roadmap.addNode(startNode)
            self._addNodeToPoseCache(startNode)
            invTraj = self._invertTrajectory(connecting_traj)
            endNode.addEdge(startNode, invTraj)
            return startNode
        return None

    def getRoadmapNodeFromPoses(self, poses, numConnectNeighbors=5, allowMoveIt=True,
                                planningTime=5.0, posTolerance=0.01, rotTolerance=0.1,
                                seedIk=None):
        iksolutions = []
        feasiblePoses = []
        # compute ik solutions for all poses and greedily try to connect
        for pose in poses:
            self._preemptionCheck()
            iksolution = self._computeIkSolution(pose, seedIk=seedIk)
            if iksolution is not None:
                feasiblePoses.append(pose)
                node = self.getRoadmapNodeFromConfig(iksolution,
                                                     numConnectNeighbors=numConnectNeighbors,
                                                     allowMoveIt=False)
                if node is not None:
                    return node
                iksolutions.append(iksolution)

        if len(iksolutions) == 0:
            rospy.logerr('Can not find any ik solutions for any of the given poses.')
            return None
        else:
            rospy.loginfo('Could not connect to any of the iks using the linear planner only.')
            rospy.loginfo('Found ' + str(len(iksolutions)) + ' out of ' + str(len(poses)) + ' ik solutions.')

        # If we reached this point, the linear planner failed at connecting
        # the roadmap to any pose -> let's try MoveIt
        # We need to start planning from somewhere, so pick the nearest node from one of the iks
        nearestNeighbors = self._roadmap.getNearestNeighbors(iksolutions[0], nNeighbors=1)
        if len(nearestNeighbors) == 0:
            raise ValueError('No nearest neighbor found for configuration ' + str(iksolutions[0]) +
                             '. The roadmap seems to be empty!')
        nn = nearestNeighbors[0]
        # Only attempt to plan to the poses that are feasible (i.e. for which we found IKs)
        (success, trajList) = self._moveItPlanningToPoses(startConfig=nn.getConfig(), poses=feasiblePoses,
                                                          planningTime=planningTime, posTolerance=posTolerance,
                                                          rotTolerance=rotTolerance)
        if success:
            if len(trajList) != 1:
                rospy.logerr('This is weird. MoveIt found a connection to one of the requested poses,' +
                             'but we have multiple trajectories. Cancelling for safety reasons.')
                return None
            # we get the configuration to which we connected from the trajectory
            config = {}
            configAsList = trajList[0].joint_trajectory.points[-1].positions
            for j in range(len(self._jointNames)):
                config[self._jointNames[j]] = configAsList[j]
            # that is our new node
            endNode = roadmap.RoadmapNode(config=config, name=self._roadmap.getNewNodeName())
            self._roadmap.addNode(endNode)
            self._addNodeToPoseCache(endNode)
            nn.addEdge(endNode, trajList[0])
            endNode.addEdge(nn, self._invertTrajectory(trajList[0]))
            return endNode
        return None

    def planToConfiguration(self, start, goal, useRoadmap=True):
        """ Attempts to plan a path from the given start configuration
            to any of the given goal configurations.
            @param start - start configuration (dict mapping joint name to value)
            @param goal - goal configuration (dict mapping joint name to value)
            @param useRoadmap - if true, uses roadmap, else only linear planner/moveit
            @return (success, traj) - tuple of boolean and a sequence of trajectories.
                    The boolean is true iff the sequence of trajectories leads to the goal.
            """
        path = None
        self._preemptionCheck()
        if useRoadmap:
            startNode = self.getRoadmapNodeFromConfig(start)
            goalNode = self.getRoadmapNodeFromConfig(goal)

            if startNode is None or goalNode is None:
                rospy.logwarn('Could not connect to the roadmap, attempting direct planning')
                return self._directPlanning(start, goal)

            path = self._roadmap.planPath(startNode, goalNode)
        if path is None:
            if useRoadmap:
                rospy.logwarn('Could not find a path using the roadmap planner. Attempting direct planning')
            return self._directPlanning(start, goal)
        return (True, path)

    def planToPoses(self, start, poses, planningTime=5.0, posTolerance=0.01, rotTolerance=0.1,
                    useRoadmap=True, seedIk=None):
        """ Attempts to plan a path from the given start configuration
            to any of the given goal poses.
            @param start - start configuration (dict mapping joint name to value)
            @param goals - list of goal poses (pose as [x,y,z, qx, qy, qz, qw]) (in base frame)
            @param useRoadmaps - if true, the roadmap is used, else direct planning is attempted.
            @param seedIk - if provided, it helps the ik solver to find an ik solution for the poses
            @return (success, traj) - tuple of boolean and trajectory. the boolean is true if a traj was found.
            """
        self._preemptionCheck()
        if useRoadmap:
            startNode = self.getRoadmapNodeFromConfig(start)
            if startNode is not None:
                goalNode = self.getRoadmapNodeFromPoses(poses, planningTime=planningTime,
                                                        posTolerance=posTolerance, rotTolerance=rotTolerance,
                                                        seedIk=seedIk)
                if goalNode is not None:
                    # Do roadmap planning if goalNode is not None
                    path = self._roadmap.planPath(startNode, goalNode)
                    if path is not None:
                        return (True, path)

            rospy.logwarn('Could not connect to the roadmap, attempting direct planning')
        return self._directPlanningToPoses(start, poses, planningTime=planningTime,
                                           posTolerance=posTolerance, rotTolerance=rotTolerance,
                                           seedIk=seedIk)

    def isPoseFeasible(self, pose, seedIk=None):
        """ Checks whether a collision-free ik solution exists for the given pose.
            @param seedIk - a seed for the inverse kinematics check
                    (it makes sense to choose the configuration from which you are planning to move to the pose.
                    if None is specified, the system will pick either the current one or another known reachable one)
                @return (bFeasible, iksolution) - (boolean whether feasible or not, ik solution if feasible)
        """
        iksolution = self._computeIkSolution(pose, seedIk)
        return (iksolution is not None, iksolution)

    def filterPosesFeasibility(self, poses, seedIk=None):
        """ Returns a subset F of poses such that all poses in F are feasible.
            A pose is feasible if a collision-free inverse kinematics solution exists.
            @param poses - a list of poses in base frame
            @param seedIk - a seed for the inverse kinematics check
                    (it makes sense to choose the configuration from which you are planning to move to the pose.
                    if None is specified, the system will pick either the current one or another known reachable one)
            @return (feasiblePoses, iksolutions) - feasiblePoses is the subset F, iksolutions
                the associated ik solutions.
        """
        feasiblePoses = []
        iksolutions = []
        for pose in poses:
            self._preemptionCheck()
            (bFeasible, iksolution) = self.isPoseFeasible(pose, seedIk)
            if bFeasible:
                feasiblePoses.append(pose)
                iksolutions.append(iksolution)
        return (feasiblePoses, iksolutions)

    def planCartesianPath(self, startConfig, goalPose, stepSize, linkName, mustReachGoal=True):
        """ Computes a path to move from the current startConfig to
            the given goal pose in a straight line in Cartesian space.
            @param startConfig - start configuration
            @param goalPose - goal pose in base frame
            @param stepSize - step size for interpolation
            @param linkName - the name of the eef link
            @param mustReachGoal - indicates whether the goal must be reached or just moving in the
                                    correct direction is good enough for success
            @return (success, traj) - tuple of boolean and trajectory. the boolean is true if a traj was found.
        """
        self._preemptionCheck()
        startJS = utils.ArgumentsCollector.dictToJointState(startConfig)
        try:
            planResult = self._cartesianLinearPathPlanner(move_group=self._moveGroup.get_name(),
                                                          link_name=linkName,
                                                          pose=goalPose,
                                                          start_config=startJS,
                                                          step_size=stepSize)
        except rospy.ROSException as err:
            rospy.logerr('planCartesianPath failed: ' + repr(err))
            return (False, None)
        return (not mustReachGoal or (planResult.success and mustReachGoal), [planResult.traj])

    def readRoadmap(self, roadmapFile):
        try:
            afile = open(roadmapFile, 'r')
            fileName, fileEnding = os.path.splitext(roadmapFile)
            if fileEnding == '.yaml':
                self._roadmap = yaml.load(afile)
            elif fileEnding == '.pickle':
                self._roadmap = pickle.load(afile)
            else:
                rospy.logerr('Unknown file ending ' + fileEnding + '. Failed to load roadmap!')
                return
            nodes = self._roadmap.getNodes()
            for node in nodes:
                self._addNodeToPoseCache(node)
            if not self._roadmap.isStronglyConnected():
                rospy.logwarn('Roadmap successfully loaded, but it is not strongly connected!')

        except IOError as err:
            rospy.logerr('Could not read roadmap file' + roadmapFile + '. Error Msg: ' + str(err))

    def computeForwardPositionKinematics(self, config):
        """ returns the eef pose [x,y,z,q1,q2,q3,q4] for the given configuration. """
        return self._armKinematics.forward_position_kinematics(config)

    def _invertTrajectory(self, traj):
        self._preemptionCheck()
        result = self._trajectoryInverter(traj=traj, move_group=self._moveGroup.get_name())
        return result.traj

    def _planPathToAny(self, start, goals, planningTime=5.0, allowMoveIt=True):
        """
            Plans a path between start and any of the goal configurations given in goals.
            Returns a tuple (traj, goalId) if successful, where traj is the trajectory and goalId
            is the index of the goal to which the trajectory leads to.
            If no trajectory is found, it returns (None, -1)
        """
        self._preemptionCheck()
        connecting_traj = None
        successfulGoal = -1
        for i in range(len(goals)):
            self._preemptionCheck()
            linPathResult = self._callLinPathPlanner(start=start, goal=goals[i])
            if linPathResult.success:
                connecting_traj = linPathResult.traj
                successfulGoal = i
                break
        # if we did not succeed with this, try connecting with moveit
        if connecting_traj is None and allowMoveIt:
            for i in range(len(goals)):
                self._preemptionCheck()
                self._moveGroup.set_start_state(self._createRobotState(start))
                try:
                    self._moveGroup.set_joint_value_target(goals[i])
                except MoveItCommanderException as moveItErr:
                    rospy.logerr('We got a MoveIt error for goal ' + str(goals[i]) + 'MoveIt says: ' + repr(moveItErr))
                    continue
                self._moveGroup.set_planning_time(planningTime)
                self._moveGroup.set_planner_id('RRTConnectkConfigDefault')
                moveit_traj = self._moveGroup.plan()

                if len(moveit_traj.joint_trajectory.points) > 0:
                    connecting_traj = moveit_traj
                    successfulGoal = i
                    break
        return (connecting_traj, successfulGoal)

    def _computeIkSolution(self, pose, seedIk=None):
        self._preemptionCheck()
        poseAsList = utils.ArgumentsCollector.poseToList(pose, euler=True)
        # Check whether we have seed IK given
        if seedIk is None:
            # By default we use the current configuration as seed for the IK query
            seedIk = self._limb.joint_angles()
            currentPose = utils.ArgumentsCollector.listPoseQuatTolistPoseEuler(
                self._armKinematics.forward_position_kinematics())

            distanceToPose = self._poseDistance(poseAsList, currentPose)
            # But let's check first whether this pose is close to annything in our roadmap
            indexSet = list(self._poseIKCache.nearest(poseAsList + poseAsList))
            # TODO: unsure what happens if the indexSet is empty (it is not a list!)
            rrConfig = self._roadmap.getNode(indexSet[0]).getConfig()
            rrDistance = self._poseDistance(poseAsList, self._armKinematics.forward_position_kinematics(rrConfig))
            if rrDistance < distanceToPose:
                seedIk = rrConfig
                # rrPose = self._armKinematics.forward_position_kinematics(rrConfig)

        self._preemptionCheck()
        response = self._inverseKinematicsSolver(move_group=self._moveGroup.get_name(),
                                                 pose=pose,
                                                 seed=utils.ArgumentsCollector.dictToJointState(seedIk))
        if not response.success:
            return None
        config = {}
        for i in range(len(response.iksolution.name)):
            jname = response.iksolution.name[i]
            # TODO: this is a bit of hack (the response contains some more joints than we need)
            if jname in self._jointNames:
                config[jname] = response.iksolution.position[i]
        return config

    def _directPlanning(self, start, goal):
        self._preemptionCheck()
        (traj, idx) = self._planPathToAny(start, [goal])
        if traj is not None:
            return (True, [traj])
        rospy.logerr('We gave our best, but we could not find any path connecting ' + str(start) + ' and ' + str(goal))
        return (False, None)

    def _directPlanningToPoses(self, startConfig, poses, planningTime=5.0,
                               posTolerance=0.01, rotTolerance=0.1, seedIk=None):
        self._preemptionCheck()
        if seedIk is None:
            seedIk = startConfig
        (feasiblePoses, iksolutions) = self.filterPosesFeasibility(poses=poses, seedIk=seedIk)
        (traj, connectConfig) = self._planPathToAny(start=startConfig, goals=iksolutions, allowMoveIt=False)
        if traj is not None:
            return (True, [traj])
        # If we reached this point, we failed using the linear planner only.
        # Try MoveIt
        return self._moveItPlanningToPoses(startConfig=startConfig, poses=feasiblePoses,
                                           planningTime=planningTime, posTolerance=posTolerance,
                                           rotTolerance=rotTolerance)

    def _moveItPlanningToPoses(self, startConfig, poses, planningTime=5.0,
                               posTolerance=0.01, rotTolerance=0.1):
        if poses is None or len(poses) == 0:
            return (False, None)
        self._preemptionCheck()
        # MoveIt planning
        self._moveGroup.set_pose_reference_frame('/base')
        self._moveGroup.set_planning_time(planningTime)
        self._moveGroup.set_start_state(self._createRobotState(startConfig))
        self._moveGroup.set_goal_position_tolerance(posTolerance)
        self._moveGroup.set_goal_orientation_tolerance(rotTolerance)
        self._moveGroup.set_planner_id('RRTConnectkConfigDefault')
        # Plan
        self._moveGroup.clear_pose_targets()
        try:
            self._moveGroup.set_pose_targets(poses)
        except MoveItCommanderException as moveItErr:
            rospy.logerr('We got a MoveIt error for all given poses. MoveIt says: ' + repr(moveItErr))
            return (False, None)
        moveit_traj = self._moveGroup.plan()
        success = len(moveit_traj.joint_trajectory.points) > 0
        solution_traj = [moveit_traj]
        return (success, solution_traj)

    def _callLinPathPlanner(self, start, goal):
        self._preemptionCheck()
        startConfig = utils.ArgumentsCollector.dictToJointState(start)
        goalConfig = utils.ArgumentsCollector.dictToJointState(goal)
        return self._linearPathPlanner(move_group=self._moveGroup.get_name(),
                                       start=startConfig,
                                       goal=goalConfig)

    def _makeJointStateMsg(self, listConfig):
        header = Header(stamp=rospy.Time.now())
        return JointState(header=header, name=self._jointNames, position=listConfig)

    def _createRobotState(self, config):
        robotState = RobotState()
        keyValuePairs = config.items()
        robotState.joint_state.name = map(lambda x: x[0], keyValuePairs)
        robotState.joint_state.position = map(lambda x: x[1], keyValuePairs)
        return robotState

    def _addNodeToPoseCache(self, node):
        pose = utils.ArgumentsCollector.listPoseQuatTolistPoseEuler(
            self._armKinematics.forward_position_kinematics(node.getConfig()))
        self._poseIKCache.insert(node.getId(), pose + pose)

    def _poseDistance(self, poseA, poseB):
        dist = 0.0
        weights = [1.0, 1.0, 1.0, 0.1, 0.1, 0.1]
        for i in range(len(poseA)):
            dist += weights[i] * pow((poseA[i] - poseB[i]), 2)
        return math.sqrt(dist)
