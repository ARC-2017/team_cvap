#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('apc_manipulation')
from utils.MoveItInterface import MoveItInterface
from actions.MoveArm import MoveArm
from actions.PickObject import PickObject
from actions.PlaceObject import PlaceObject
from actions.ApproachArm import ApproachArm
from actions.RetreatArm import RetreatArm
from geometry_msgs.msg import PoseStamped
import sys

if __name__ == "__main__":
    rospy.init_node('manipulation_node')
    pathToConfigFile = rospy.get_param('/apc/manipulation/named_configs')
    pathToSeedsFile = rospy.get_param('/apc/manipulation/ik_seeds')
    pathToRoadmapLeft = rospy.get_param('/apc/manipulation/roadmap_left')
    pathToRoadmapRight = rospy.get_param('/apc/manipulation/roadmap_right')
    leftHandElectricGripper = rospy.get_param('/apc/manipulation/left_electric_gripper', False)
    moveItInterface = MoveItInterface(sys.argv, configFileName=pathToConfigFile,
                                      ikseedsFileName=pathToSeedsFile,
                                      roadmapLeft=pathToRoadmapLeft,
                                      roadmapRight=pathToRoadmapRight,
                                      leftElectricGripper=leftHandElectricGripper)
    initSuccess = moveItInterface.init()
    if not initSuccess:
        rospy.logerr('Could not initialize MoveIt. Aborting.')
        sys.exit(-1)

    # attach collision models of IDS cameras
    ids_camera_dimensions = [0.04, 0.05, 0.03]
    ids_camera_pose = PoseStamped()
    ids_camera_pose.header.frame_id = 'right_gripper_base'
    ids_camera_pose.pose.position.x = 0.04
    ids_camera_pose.pose.position.y = 0.0
    ids_camera_pose.pose.position.z = 0.035
    ids_camera_pose.pose.orientation.x = 0.0
    ids_camera_pose.pose.orientation.y = 0.0
    ids_camera_pose.pose.orientation.z = 0.0
    ids_camera_pose.pose.orientation.w = 1.0

    arms = ['left', 'right']
    touchLinks = ['']
    for arm in arms:
        if arm=='left' and leftHandElectricGripper:
            continue

        if arm=='left':
            touchLinks = ['left_gripper_base', 'left_gripper_extension',
                          'left_hand', 'left_hand_camera', 'left_hand_accelerometer',
                          'left_hand_range', 'left_wrist']

        elif arm=='right':
            touchLinks = ['right_gripper_base', 'right_gripper_extension',
                          'right_hand', 'right_hand_camera', 'right_hand_accelerometer',
                          'right_hand_range', 'right_wrist']

        ids_camera_pose.header.frame_id = arm + '_gripper_base'
        moveItInterface.attachBoxObject(objectName='ids_' + arm,
                                        linkName=arm + '_gripper_base',
                                        touchLinks=touchLinks,
                                        poseStamped=ids_camera_pose,
                                        objectDimensions=ids_camera_dimensions)

        rospy.sleep(1.0)


    approachService = ApproachArm("/apc/manipulation/approach")
    retreatService = RetreatArm("/apc/manipulation/retreat")
    moveArmService = MoveArm('/apc/manipulation/move_arm', moveItInterface)
    pickingService = PickObject('/apc/manipulation/pick_object', moveItInterface)
    placingService = PlaceObject('/apc/manipulation/place_object', moveItInterface)

    rospy.loginfo('Started node /manipulation_node')
    rospy.spin()
