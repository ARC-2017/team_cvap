#!/usr/bin/env python
"""
    This class provides synchronized access to MoveIt.

    @author: Joshua Haustein (haustein@kth.se)
"""
# general python includes
import yaml
import threading
from subprocess import check_output, CalledProcessError
# ROS includes
import rospy
import moveit_commander
import rospkg
import tf
from std_srvs.srv import Empty
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, MoveGroupAction
from tf_conversions import posemath
from geometry_msgs.msg import PoseStamped
import actionlib
# Baxter
import baxter_interface
from baxter_interface.srv import ControlMode
# APC includes
import utils.ArgumentsCollector
import utils.APCGlobalPathPlanner
import PyKDL as kdl
import utils.TFCache

MAX_SLEEP_TIME = 0.5


class PreemptionException(Exception):
    def __init__(self, value):
        self._value = value

    def __str__(self):
        return 'PreemptionException: Aborting execution - value:' + repr(self._value)


class MoveItProblemException(Exception):
    def __init__(self, value):
        self._value = value

    def __str__(self):
        return 'MoveItProblemException: Aborting execution because sth is wrong with the move_group node - value:' + repr(self._value)


class MoveItMissingException(MoveItProblemException):
    def __init__(self, value):
        self._value = value

    def __str__(self):
        return 'MoveItMissingException: Aborting execution because there is no move_group node - value:' + repr(self._value)


class MoveItOverloadException(MoveItProblemException):
    def __init__(self, value):
        self._value = value

    def __str__(self):
        return 'MoveItOverladException: Aborting execution because there are multiple instances of move_group - value:' + repr(self._value)


class MoveResult:
    PlanFail, ExecuteFail, Success = range(3)


class APCBaxterGripper(object):
    def __init__(self, baxter_gripper, electric=False):
        self._gripper = baxter_gripper
        self._electric = electric

    def pick(self):
        if self._electric:
            self._gripper.close()
        else:
            self._gripper.command_suction()

    def isPicking(self):
        if self._electric:
            if self._gripper.gripping():
                self._gripper.close()
            return self._gripper.gripping()
        else:
            return self._gripper.vacuum()

    def stop(self):
        if self._electric:
            self._gripper.open()
        else:
            self._gripper.stop()

    def calibrate(self):
        if self._electric:
            self._gripper.calibrate()

    def isElectric(self):
        return self._electric


class MoveItInterface(object):
    """ Interface to MoveIt. Get the lock before doing anything and
        release it as soon as you don't use it anymore! You can not execute
        any of the method without acquiring the lock. This prevents multiple
        threads from sending commands to MoveIt."""

    def __init__(self, argv, configFileName=None, ikseedsFileName=None, roadmapLeft=None, roadmapRight=None,
                 leftElectricGripper=False, rightElectricGripper=False):
        self._lock = threading.RLock()
        moveit_commander.roscpp_initialize(argv)
        self._moveGroupPid = None
        # wait for move_group action to become available to avoid timeout
        # move_group_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        # rospy.loginfo('[apc_grasping/MoveItInterface]: waiting for move_group action server')
        # move_group_client.wait_for_server()
        self._robotCommander = None
        self._leftArmGroup = None
        self._moveitScene = None
        self._rightArmGroup = None
        self._preempted = False
        self._tfListener = utils.TFCache.TFCache()
        self._objectsInScene = []
        moveGroupAvailable = False
        self._configurationStack = {'left_arm': [], 'right_arm': []}
        # check for move group availability (the ugly way....)
        while not moveGroupAvailable and not rospy.is_shutdown():
            try:
                self.checkMoveGroup(bResetScene=False)
                moveGroupAvailable = True
            except MoveItMissingException:
                rospy.logwarn('Waiting for move_group')
                self.sleep(0.5)

        self._co_publisher = rospy.Publisher('/collision_object', CollisionObject, queue_size=1)
        self._co_attach_publisher = rospy.Publisher('/attached_collision_object',
                                                    AttachedCollisionObject, queue_size=1)

        self._leftGlobalPlanner = utils.APCGlobalPathPlanner.APCGlobalPathPlanner('left_arm',
                                                                                  self._leftArmGroup,
                                                                                  self.checkPreemption)
        self._rightGlobalPlanner = utils.APCGlobalPathPlanner.APCGlobalPathPlanner('right_arm',
                                                                                   self._rightArmGroup,
                                                                                   self.checkPreemption)
        self._armSwitchControllerService = {}
        rospy.wait_for_service('/apc/hacks/set_control_mode_left')
        self._armSwitchControllerService['left_arm'] = rospy.ServiceProxy('/apc/hacks/set_control_mode_left', ControlMode)
        rospy.wait_for_service('/apc/hacks/set_control_mode_right')
        self._armSwitchControllerService['right_arm'] = rospy.ServiceProxy('/apc/hacks/set_control_mode_right', ControlMode)
        rospy.wait_for_service('/pause_joint_trajectory_server_left')
        rospy.wait_for_service('/pause_joint_trajectory_server_right')
        self._pause_joint_trajectory_service_left = rospy.ServiceProxy('pause_joint_trajectory_server_left', Empty)
        self._pause_joint_trajectory_service_right = rospy.ServiceProxy('pause_joint_trajectory_server_right', Empty)
        # self._tfListener = tf.TransformListener()
        self._leftGripper = APCBaxterGripper(baxter_interface.Gripper('left'), electric=leftElectricGripper)
        self._rightGripper = APCBaxterGripper(baxter_interface.Gripper('right'), electric=rightElectricGripper)
        self._leftArm = baxter_interface.Limb('left')
        self._rightArm = baxter_interface.Limb('right')
        self._head = baxter_interface.Head()
        self._namedConfigurations = {}
        if roadmapLeft is not None:
            rospy.loginfo('Loading roadmap for left arm from path ' + roadmapLeft)
            rospy.loginfo('Note, this may take a while')
            self._leftGlobalPlanner.readRoadmap(roadmapLeft)
            rospy.loginfo('Roadmap successfully loaded!')
        if roadmapRight is not None:
            rospy.loginfo('Loading roadmap for right arm from path ' + roadmapRight)
            rospy.loginfo('Note, this may take a while')
            self._rightGlobalPlanner.readRoadmap(roadmapRight)
            rospy.loginfo('Roadmap successfully loaded!')

        self._seedIks = {}
        if ikseedsFileName is not None:
            try:
                ikseedFile = open(ikseedsFileName, 'r')
                self._seedIks = yaml.load(ikseedFile)
                ikseedFile.close()
            except IOError as err:
                rospy.logerr('Could not load ik seeds file: ' + repr(err))
        if 'left_arm' not in self._seedIks or self._seedIks['left_arm'] is None:
            self._seedIks['left_arm'] = {}
        if 'right_arm' not in self._seedIks or self._seedIks['right_arm'] is None:
            self._seedIks['right_arm'] = {}

        if configFileName is not None:
            try:
                configFile = open(configFileName, 'r')
                self._namedConfigurations = yaml.load(configFile)
                configFile.close()
            except IOError as err:
                rospy.logerr('Could not load configurations from file ' + configFileName)
        if 'left_arm' not in self._namedConfigurations:
            self._namedConfigurations['left_arm'] = {}
        if 'right_arm' not in self._namedConfigurations:
            self._namedConfigurations['right_arm'] = {}
        rospy.loginfo('MoveItInterface created.')

    def checkMoveGroup(self, bResetScene=True):
        with self._lock:
            try:
                pidofOutput = check_output(['pidof', 'move_group'])
                pidList = map(int, pidofOutput.split())
                if len(pidList) > 1:
                    rospy.logerr('There are multiple instances of MoveIt running: ' + repr(pidList))
                    raise MoveItOverloadException(repr(pidList))
                if len(pidList) == 1:
                    currentMoveGroupPid = pidList[0]
                    if self._moveGroupPid != currentMoveGroupPid:
                        # There is a risk of getting stuck here and not preempting :/
                        move_group_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
                        rospy.loginfo('[apc_grasping/MoveItInterface]: waiting for move_group action server')
                        move_group_client.wait_for_server()
                        self._robotCommander = moveit_commander.RobotCommander()
                        self._leftArmGroup = moveit_commander.MoveGroupCommander("left_arm")
                        self._moveitScene = moveit_commander.PlanningSceneInterface()
                        self._rightArmGroup = moveit_commander.MoveGroupCommander("right_arm")
                        self._moveGroupPid = currentMoveGroupPid
                        if bResetScene:
                            self.resetPlanningScene()
                else:
                    rospy.logerr('There is no move_group!')
                    raise MoveItMissingException()
            except CalledProcessError as e:
                rospy.logerr('Could not check whether MoveIt is still around: ' + repr(e))
                raise MoveItMissingException('Failed to check it!')

    def acquireLock(self):
        """ Acquire this lock before doing anything with MoveIt! """
        # TODO we could alternatively block instead, but that could lead to
        # deadlocks.
        gotLock = self._lock.acquire(False)
        print 'Acquired lock: ', threading.current_thread().ident
        if not gotLock:
            raise ValueError('Could not acquire lock for MoveIt access.' +
                             'This should not happen and probably means that multiple ' +
                             ' actions are executed in parallel.')

    def releaseLock(self):
        """ Release this lock as soon as other threads are allowed to modify MoveIt!"""
        print 'Releasing lock: ', threading.current_thread().ident
        self._lock.release()

    def setPreempted(self, value):
        self._preempted = value

    def checkPreemption(self):
        if self._preempted:
            raise PreemptionException('MoveItInterface::checkPreemption')

    def checkProcessState(self):
        self.checkPreemption()
        self.checkMoveGroup()

    def init(self):
        """ Initializes the robot and moveit."""
        with self._lock:
            rospy.loginfo('Initializing MoveIt and Baxter.')
            self._leftGripper.calibrate()
            self._rightGripper.calibrate()
            try:
                self._task = rospy.get_param('/apc/task')
            except rospy.ROSException:
                rospy.logerr('Could not initialize, missing /apc/task parameter which specifies pick/stow task')
                success = False
                return success

            try:
                self._collisionObjects = rospy.get_param('/apc/manipulation/collision_objects')
            except rospy.ROSException:
                rospy.logerr('Could not initialize, missing collision_objects param')
                success = False
                return success

            try:
                self._collisionMeshesRootDir = rospkg.RosPack().get_path('apc_2016_mesh_models') + '/object_models/'
            except rospy.ROSException:
                rospy.logerr('Could not find path to apc_2016_mesh_models package')
                success = False
                return success

            success = self.resetPlanningScene()

            if success:
                rospy.loginfo('Successfully initialized!')
            else:
                rospy.logerr('Could not initialize.')
            return success

    def resetPlanningScene(self):
        """ Reset the planning scene. Blocks until either the scene is successfully reset or
            the ros node is killed.
            NOTE: Acquire the lock before calling this function!
            @return True iff the scene is successfully reset."""
        with self._lock:
            self.checkMoveGroup(bResetScene=False)
            self._moveitScene.remove_world_object()
            self._objectsInScene = []
            success = False
            hasTransform = False
            hasMesh = False

            while not rospy.is_shutdown() and not success:
                for cObject in self._collisionObjects:
                    self.checkPreemption()
                    hasTransform = False
                    hasMesh = False
                    while not hasTransform and not rospy.is_shutdown():
                        self.checkPreemption()
                        try:
                            (trans, rot) = self._tfListener.lookupTransform('/base', cObject['name'], rospy.Time(0))
                            hasTransform = True
                            cObject['pose'] = utils.ArgumentsCollector.createROSPose(trans, rot)

                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rospy.logwarn('Could not receive ' + cObject['name'] + ' pose.')
                            hasTransform = False
                            rospy.sleep(1.0)

                    while not hasMesh and not rospy.is_shutdown():
                        self.checkPreemption()
                        try:
                            self._objectsInScene.append(cObject['name'])
                            self._moveitScene.add_mesh(
                                # pose of ROSPose type
                                cObject['name'], cObject['pose'], self._collisionMeshesRootDir + cObject['mesh_path'])
                            hasMesh = True
                            success = True
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            errorMsg = 'Mesh stored at %s  is invalid. Please fix this!' % (self._collisionMeshesRootDir + cObject['mesh_path'])
                            rospy.logerr(errorMsg)
                            hasMesh = False
                            rospy.sleep(1.0)
                success = True
            return success


    def addObjectsToPlanningScene(self, objectNames):
        """ Adds the given set of objects to the planning scene.
            If any of the objects is already in the planning scene, its
            pose is updated to the most recent one.

            @param objectNames - a list of object names to add. """
        with self._lock:
            self.checkProcessState()
            for objectName in objectNames:
                objectPose = self.getObjectTransform(objectName)
                if objectName in self._objectsInScene:
                    self._moveObject(objectName=objectName, objectPose=objectPose)
                else:
                    self._addObject(objectName=objectName, objectPose=objectPose)

    def attachBoxObject(self, objectName, linkName, touchLinks, poseStamped, objectDimensions):
        """ Attach the specified box object to the given linkName of an arm with given poseStamped and objectDimensions
        (dimensions of the box).
            @param objectName - name of the object to attach (str)
            @param linkName - link to attach the collision model to (str)
            @param touchLinks - links for which to disable collisions with the attached object (list of str)
            @param poseStamped - stamped pose of the object (PoseStamped)
            @param objectDimensions - dimensions of the box object (list of float), 3 elements
            @param armName - name of the arm (either left_arm or right_arm)
        """

        self._moveitScene.attach_box(linkName, objectName, poseStamped, objectDimensions, touchLinks)

    def resetConfigurationStack(self, arm):
        with self._lock:
            if arm in self._configurationStack:
                self._configurationStack[arm] = []

    def stackCurrentConfiguration(self, arm):
        with self._lock:
            if arm in self._configurationStack:
                self._configurationStack[arm].append(self.getCurrentConfiguration(arm))

    def popConfigurationStack(self, arm):
        with self._lock:
            if arm in self._configurationStack:
                if len(self._configurationStack[arm]) > 0:
                    self._configurationStack[arm].pop()

    def retreatConfigurationStack(self, arm):
        with self._lock:
            if arm in self._configurationStack:
                stack = self._configurationStack[arm]
                limb = self.getBaxterArm(arm)
                rospy.logwarn('Retreating using the configuration stack')
                if arm == 'left_arm':
                    pause_jts = self._pause_joint_trajectory_service_left
                else:
                    pause_jts = self._pause_joint_trajectory_service_right
                pause_jts()
                while len(stack) > 0:
                    rospy.logwarn('Going to next configuration. ' + str(len(stack)) + ' configurations left.' )
                    limb.move_to_joint_positions(stack.pop())
                pause_jts()

    def releaseObject(self, objectName):
        """ Releases the specified object."""
        # TODO
        pass

    def getNamedConfiguration(self, name, arm):
        if arm in self._namedConfigurations:
            armConfigs = self._namedConfigurations[arm]
            if name in armConfigs:
                return armConfigs[name]
        rospy.logwarn('The configuration ' + name + ' for ' + arm + ' is unknown.')
        return None

    def getConfigurationName(self, binName, typ, special):
        return binName + '_' + typ + '_' + special

    def getCurrentConfiguration(self, arm):
        # with self._configurationLock:
        if arm == 'left_arm':
            return self._leftArm.joint_angles()
        elif arm == 'right_arm':
            return self._rightArm.joint_angles()
        raise ValueError('Unknown arm ' + arm + '. Can not retrieve configuration.')

    def getBaxterArm(self, arm):
        with self._lock:
            if arm == 'left_arm':
                return self._leftArm
            elif arm == 'right_arm':
                return self._rightArm
            else:
                rospy.logerr('Unknown arm requested %s') % arm
                return None

    def getGlobalPlanner(self, arm):
        with self._lock:
            if arm == 'left_arm':
                return self._leftGlobalPlanner
            elif arm == 'right_arm':
                return self._rightGlobalPlanner
            else:
                rospy.logerr('Unknown planner requested for arm %s') % arm
                return None

    def getIKSeed(self, arm, poseName):
        """ Checks whether for a given arm and given named pose, we have
            we have a seed for the inverse kinematics solver
            @return ik seed if available, else None
        """
        with self._lock:
            if poseName in self._seedIks[arm]:
                return self._seedIks[arm][poseName]
            else:
                return None

    def removeObjectsFromPlanningScene(self, objectNames):
        """ Removes the given objects from the planning scene.
            @param objectNames: a list of object names to remove. """
        with self._lock:
            self.checkProcessState()
            objectsToRemove = []
            for objectName in objectNames:
                if objectName in self._objectsInScene:
                    self._planningScene.remove_world_object(objectName)
                    objectsToRemove.append(objectName)
                else:
                    rospy.logwarn("Attempting to remove object " + objectName + " from planning scene" +
                                  " although it shouldnt be there.")

            self._objectsInScene = [x for x in self._objectsInScene if x not in objectsToRemove]

    def moveToPosition(self, pos, arm, execute=True):
        # moveGroup.set_position_target(position)
        pass

    def moveToPose(self, pose, arm, posTolerance=0.02, rotTolerance=0.1, execute=True, planningTime=5.0,
                   useRoadmap=True, controlMode='position_w_id', seedIk=None,
                   startConfig=None):
        return self.moveToPoses(poses=[pose], arm=arm, posTolerance=posTolerance, rotTolerance=rotTolerance,
                                execute=execute, planningTime=planningTime, useRoadmap=useRoadmap, seedIk=seedIk,
                                controlMode=controlMode, startConfig=startConfig)

    def moveToPoses(self, poses, arm, posTolerance=0.01, rotTolerance=0.1, execute=True,
                    planningTime=5.0, useRoadmap=True, seedIk=None, controlMode='position_w_id',
                    startConfig=None):
        """ Plans/moves the specified arm to one of the given poses.
            @param poses - goal poses of type PoseStamped. Can be defined in any known frame.
            @param arm - either left_arm or right_arm
            @param execute - if true, try to execute, otherwise just plan (automatically set to false if startConfig is specified)
            @param useRoadmap - if true, uses the roadmap, else not
            @param seedIk - if provided, the ik seed helps the ik solver to find an ik solution for the poses
            @param startConfig - start configuration (optional, default is current configuration)
            @return (feedback, trajectory), where feedback is one of the values in MoveResult
                     and trajectory is the MoveIt trajectory.
        """
        with self._lock:
            self.checkProcessState()
            self.moveHeadAside(arm)
            convertedPoses = self._convertPosesToFrame(poses, goalFrame='base')
            if convertedPoses is None:
                return (MoveResult.Fail, None)

            moveGroup = self.getMoveGroup(arm)
            globalPlanner = self.getGlobalPlanner(arm)
            if startConfig is None:
                startConfig = self.getCurrentConfiguration(arm)
            else:
                execute = False
            (planningSuccess, trajList) = globalPlanner.planToPoses(start=startConfig,
                                                                    poses=convertedPoses, useRoadmap=useRoadmap,
                                                                    seedIk=seedIk)

            if planningSuccess and execute:
                return self._executeTrajectoryList(trajList, moveGroup, controlMode=controlMode)
            elif planningSuccess and not execute:
                return (MoveResult.Success, trajList)

            errorMsg = 'Could not find a path to ' + repr(len(convertedPoses)) + ' poses.'
            rospy.logerr(errorMsg)
            return (MoveResult.PlanFail, None)

    def moveToConfiguration(self, config, arm, execute=True, planningTime=5, useRoadmap=True,
                            controlMode='position_w_id', noPlanning=False, startConfig=None):
        """ Plans/moves the specified arm to the given configuration.
            @param config - configuration to move to (dictionary name -> value).
            @param arm - either left_arm or right_arm
            @param execute - if true, try to execute, otherwise just plan (automatically set to false if startConfig is specified)
            @param startConfig - optional, if None, the current configuration is used (only active when planning is true)
            @return (feedback, trajectory), where feedback is one of the values in MoveResult
                     and trajectory is the MoveIt trajectory.
        """
        with self._lock:
            self.checkProcessState()
            if config is None or not isinstance(config, dict):
                if len(config) != 7:
                    raise ValueError('Invalid argument: ' + repr(config))
            if startConfig is None:
                startConfig = self.getCurrentConfiguration(arm)
            else:
                execute = False
            if utils.APCGlobalPathPlanner.configurationsAreIdentical(config, startConfig):
                return (MoveResult.Success, [])

            if noPlanning:
                self._moveArmBlindly(config, arm)
                return (MoveResult.Success, None)

            self.moveHeadAside(arm)
            moveGroup = self.getMoveGroup(arm)
            globalPlanner = self.getGlobalPlanner(arm)
            (planningSuccess, trajList) = globalPlanner.planToConfiguration(start=startConfig,
                                                                            goal=config, useRoadmap=useRoadmap)
            if not planningSuccess:
                return (MoveResult.PlanFail, None)

            if not execute:
                return (MoveResult.Success, trajList)

            return self._executeTrajectoryList(trajList, moveGroup, controlMode=controlMode)

    def moveRelative(self, arm, x=0.0, y=0.0, z=0.0, rx=0.0, ry=0.0, rz=0.0, frameId=None,
                     interpolate=False, stepSize=0.02, execute=True, mustReachGoal=True,
                     controlMode='position_w_id', startConfig=None):
        """ Move the end effector relative to its current pose.
            @param arm - which arm (left_arm or right_arm)
            @param x,y,z,rx,ry,rz - position and orientation (in roll, pitch, yaw)
            @param frameId - relative to which frame to (None means EEF)
            @param interpolate - if true, we attempt to move on a straight line in cartesian space
                                Note that this means, we can not use the roadmap.
            @param stepSize - step size for straight line motion. The smaller the more accurate the
                            line at the cost of slower execution.
            @param mustReachGoal - indicates whether MoveResult.Success should only be returned
                    if the goal pose can be reached. If false, the longest valid trajectory towards
                    the goal is returned. Note this option is only considered if interpolate=True.
            @param startConfig - start configuration from where to plan from. If set, execute is set to
                    False, if not set the current configuration is used as start configuration.
        """
        with self._lock:
            self.checkProcessState()
            if frameId is None:
                frameId = self.getGripperFrameName(arm)
            if startConfig is None:
                startConfig = self.getCurrentConfiguration(arm)

            globalPlanner = self.getGlobalPlanner(arm)
            # express eef pose in frameId
            eefPoseInBase = globalPlanner.computeForwardPositionKinematics(startConfig)
            F_base_eef = kdl.Frame(kdl.Rotation.Quaternion(*eefPoseInBase[3:7]), kdl.Vector(*eefPoseInBase[0:3]))
            F_base_frame = posemath.fromMsg(self.getTransform(frameId, 'base').pose)
            F_frame_eef = F_base_frame.Inverse() * F_base_eef
            F_rel = kdl.Frame(kdl.Rotation.RPY(rx, ry, rz), kdl.Vector(x, y, z))
            F_frame_eef = F_rel * F_frame_eef

            (position, rotation) = posemath.toTf(F_frame_eef)
            goalPose = utils.ArgumentsCollector.createROSPose(position=position, rotation=rotation, frame_id=frameId)
            convertedPoses = self._convertPosesToFrame([goalPose], goalFrame='base')
            if interpolate:
                self.moveHeadAside(arm)
                moveGroup = self.getMoveGroup(arm)
                (planningSuccess, trajList) = globalPlanner.planCartesianPath(startConfig=startConfig,
                                                                              goalPose=convertedPoses[0],
                                                                              stepSize=stepSize,
                                                                              linkName=self.getGripperFrameName(arm),
                                                                              mustReachGoal=mustReachGoal)
                if not planningSuccess:
                    return (MoveResult.PlanFail, None)
                elif not execute:
                    return (MoveResult.Success, trajList)
                else:
                    return self._executeTrajectoryList(trajList, moveGroup, controlMode=controlMode)
            return self.moveToPoses([goalPose], arm, startConfig=startConfig)

    def moveHeadAside(self, arm):
        with self._lock:
            pan = 1.5
            if arm == 'left_arm':
                pan = -1.5
            self._head.set_pan(pan)

    def executeTraj(self, armName, trajList):
        with self._lock:
            return self._executeTrajectoryList(trajList, self.getMoveGroup(armName))[0]

    def getCurrentHeadSide(self):
        pan = self._head.pan()
        if pan < 0.0:
            return 'right'
        return 'left'

    def getMoveGroup(self, name, clearGroup=True):
        """ Returns the specified MoveGroup.
            Currently left_arm and right_arm are supported.
            @param name - either left_arm or right_arm
            @param clearGroup - if true, clear targets and constraints"""
        with self._lock:
            self.checkProcessState()
            moveGroup = None
            if name == 'left_arm':
                moveGroup = self._leftArmGroup
            elif name == 'right_arm':
                moveGroup = self._rightArmGroup
            else:
                raise ValueError('Move group ' + name + ' is currently not supported!')
            if clearGroup:
                moveGroup.clear_path_constraints()
                moveGroup.clear_pose_targets()
            return moveGroup

    def getGripper(self, name):
        with self._lock:
            if name == 'left':
                return self._leftGripper
            elif name == 'right':
                return self._rightGripper
            else:
                raise ValueError('Unknown gripper ' + name)

    def getArmKinematics(self, name):
        with self._lock:
            if name == 'left_arm':
                return self._leftArmKinematics
            elif name == 'right_arm':
                return self._rightArmKinematics
            else:
                raise ValueError('Unknown arm ' + name)

    def getGripperName(self, armName):
        if armName == 'left_arm':
            return 'left'
        elif armName == 'right_arm':
            return 'right'
        raise ValueError('Unknown gripper ' + armName)

    def getGripperFrameName(self, armName):
        return self.getGripperName(armName) + '_gripper'

    def getGripperBaseFrameName(self, armName):
        return self.getGripperFrameName(armName) + '_base'

    def getObjectFrameName(self, objectName):
        """ The frame of objectName is assumed to be $objectName_final. """
        return objectName + '_final'

    def sleep(self, duration):
        numSleepTimes = duration / MAX_SLEEP_TIME
        sleepRemainer = duration - numSleepTimes * MAX_SLEEP_TIME
        for i in range(int(numSleepTimes)):
            self.checkPreemption()
            rospy.sleep(MAX_SLEEP_TIME)
        self.checkPreemption()
        rospy.sleep(sleepRemainer)

    def getObjectTransform(self, objectName, base_frame='/base'):
        """ Returns the transform of the given object.
            This is a convenience function that calls self.getTransform.
        """
        return self.getTransform(frame_id=self.getObjectFrameName(objectName), base_frame=base_frame)

    def getTransform(self, frame_id, base_frame='/base'):
        """ Returns the transform from frame_id to base_frame, if known."""
        if not self._tfListener.frameExists(frame_id) or not self._tfListener.frameExists(base_frame):
            # rospy.logerr('At least on of these frames is unknown: ' + frame_id + ', ' + base_frame)
            raise ValueError('At least one of these frames is unknown ' + frame_id + ', ' + base_frame)
        #time = self._tfListener.getLatestCommonTime(frame_id, base_frame)
        rosPose = None
        for trial in range(10):
                self.checkPreemption()
                try:
                    position, orientation = self._tfListener.lookupTransform(base_frame, frame_id, rospy.Time(0))
                    rosPose = utils.ArgumentsCollector.createROSPose(position, orientation, base_frame)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.sleep(0.5)
        if rosPose is None:
            raise RuntimeError('We could not retrieve the transform from ' + base_frame + ' to ' + frame_id)
        return rosPose

    def isPoseFeasible(self, pose, arm, seedConfig=None):
        """
            Returns whether the given pose is feasible or not.
            A pose is feasible iff a collision-free ik solution exists.
            @param pose - stamped pose
            @param arm - the arm for which to test feasibility (left_arm or right_arm)
            @param seedConfig(optional) - if not None, the ik solver is seeded with this config
            @return (bFeasible, iksolution) - (whether it is feasible or not, ik solution if feasible)
        """
        self.checkProcessState()
        convertedPoses = self._convertPosesToFrame([pose], '/base')
        if convertedPoses is None:
            return (False, None)
        if arm == 'left_arm':
            return self._leftGlobalPlanner.isPoseFeasible(convertedPoses[0], seedIk=seedConfig)
        elif arm == 'right_arm':
            return self._rightGlobalPlanner.isPoseFeasible(convertedPoses[0], seedIk=seedConfig)
        else:
            raise ValueError('Unknown arm: ' + arm + '! Can not determine feasibility!')

    def isPoseKnown(self, poseName):
        return self._tfListener.frameExists(poseName)

    def getFeasiblePoses(self, poses, arm, seedConfig=None):
        """
            Filters the given set of poses based on feasibility.
            A pose is feasible iff a collision-free ik solution exists.
            @param poses - a list of poses
            @param arm - the arm for which to test feasibility (left_arm or right_arm)
            @param seedConfig(optional) - if not None, the ik solver is seeded with this config
            @return (filteredPoses, iksolutions) - filtered poses and their respective ik solutions
        """
        self.checkProcessState()
        convertedPoses = self._convertPosesToFrame(poses, '/base')
        if convertedPoses is None:
            return ([], [])
        if arm == 'left_arm':
            (feasiblePoses, iksolutions) = self._leftGlobalPlanner.filterPosesFeasibility(convertedPoses,
                                                                                          seedIk=seedConfig)
        elif arm == 'right_arm':
            (feasiblePoses, iksolutions) = self._rightGlobalPlanner.filterPosesFeasibility(convertedPoses,
                                                                                           seedIk=seedConfig)
        else:
            raise ValueError('Unknown arm: ' + arm + '! Can not filter poses based on feasibility!')
        resultingPoses = map(utils.ArgumentsCollector.makeStamped, feasiblePoses)
        return (resultingPoses, iksolutions)

    def transformPose(self, pose, targetFrame):
        with self._lock:
            tTargetFramePoseFrame = None
            for trial in range(10):
                self.checkPreemption()
                try:
                    tTargetFramePoseFrame = self._tfListener.lookupTransform(targetFrame, pose.header.frame_id, rospy.Time(0))
                    break
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.sleep(0.5)
            if tTargetFramePoseFrame is None:
                raise ValueError('Could not transform pose from ' + pose.header.frame_id + ' to ' + targetFrame)
            # Convert poses to KDL
            fTargetFramePoseFrame = posemath.fromTf(tTargetFramePoseFrame)
            fPoseFrameChild = posemath.fromMsg(pose.pose)
            # Compute transformation
            fTargetFrameChild = fTargetFramePoseFrame * fPoseFrameChild
            # Convert back to ROS message
            poseStampedTargetFrameChild = PoseStamped()
            poseStampedTargetFrameChild.pose = posemath.toMsg(fTargetFrameChild)
            poseStampedTargetFrameChild.header.stamp = rospy.Time.now()
            poseStampedTargetFrameChild.header.frame_id = targetFrame
            return poseStampedTargetFrameChild

    def moveObject(self, objectName, objectPose):
        with self._lock:
            co = CollisionObject()
            co.operation = co.MOVE
            co.id = objectName
            co.mesh_poses = [objectPose.pose]
            co.header.stamp = objectPose.header.stamp
            co.header.frame_id = objectPose.header.frame_id
            self._co_publisher.publish(co)
    # def approachEndEffector(self, armName, maxDistance):
    #     """Command the given arm's end effector to move forward until contact."""
    #     goal = ApproachActionGoal()
    #     if armName != 'left_arm' and armName != 'right_arm':
    #         raise ValueError('Unknown gripper ' + armName)
    #
    #     goal.goal.arm = armName
    #     goal.goal.max_distance = maxDistance
    #     self._approach_client.send_goal(goal)
    #
    #     success = self._approach_client.wait_for_result(rospy.Duration(self._approach_timeout))
    #
    #     if success:
    #         state = self._approach_client.get_state()
    #
    #         if state != SUCCEEDED:
    #             raise ValueError('Approach client did not succeed')
    #
    #     else:
    #         self._approach_client.cancel_all_goals()
    #         raise ValueError('Preempted the approach action due to timeout')
    #
    #     return True


    # THE FOLLOWING METHODS ARE PRIVATE AND SHOULD NOT BE CALLED FROM OUTSIDE OF THIS CLASS!
    def _addObject(self, objectName, objectPose):
        meshPath = self._getMeshPath(objectName)
        self._moveitScene.add_mesh(objectName, objectPose, meshPath)


    def _getMeshPath(self, objName):
        meshPath = ""
        receivedPath = False
        parameterKey = '/apc/' + objName + '_mesh_path'
        while not rospy.is_shutdown() and not receivedPath:
            try:
                meshPath = rospy.get_param(parameterKey)
                receivedPath = True
            except rospy.ROSException:
                rospy.logerr('Could not acquire path for mesh from parameter server.' +
                             ' Please add the path to the mesh of ' + objName + ' to the parameter server ' +
                             '(parameter name:' + parameterKey + ' )! ')
                rospy.sleep(1.0)
        return meshPath

    def _executeTrajectoryList(self, traj_list, moveGroup, controlMode=None):
        for traj in traj_list:
            self.checkProcessState()
            if utils.APCGlobalPathPlanner.isTrajectoryTrivial(traj):
                rospy.logwarn('The trajectory is trivial. In order to prevent jerky motions, switch controller mode to position.')
                self._armSwitchControllerService[moveGroup.get_name()](mode="position")
            else:
                if controlMode is None:
                    controlMode = 'position_w_id'
                rospy.loginfo('The trajectory is normal. Using ' + controlMode + '  mode.')
                self._armSwitchControllerService[moveGroup.get_name()](mode=controlMode)

            success = moveGroup.execute(traj)
            rospy.sleep(0.5)
            if not success:
                return (MoveResult.ExecuteFail, traj_list)
        return (MoveResult.Success, traj_list)

    def _convertPosesToFrame(self, poses, goalFrame='/base'):
        convertedPoses = []
        for pose in poses:
            self.checkPreemption()
            if not isinstance(pose, PoseStamped):
                raise ValueError('Invalid argument: pose must be of type PoseStamped.')
            tpose = self.transformPose(pose, goalFrame)
            if tpose is None:
                errorMsg = 'Could not transform pose to base frame: ' + repr(pose)
                rospy.logerr(errorMsg)
                return None
            convertedPoses.append(tpose.pose)
        return convertedPoses

    def _moveArmBlindly(self, config, arm):
        limb = self.getBaxterArm(arm)
        rospy.logwarn('Moving the arm without planning')
        if arm == 'left_arm':
            pause_jts = self._pause_joint_trajectory_service_left
        else:
            pause_jts = self._pause_joint_trajectory_service_right
        pause_jts()
        limb.move_to_joint_positions(config)
        pause_jts()
