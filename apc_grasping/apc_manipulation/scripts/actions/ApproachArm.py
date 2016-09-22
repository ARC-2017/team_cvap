#! /usr/bin/env python

import roslib
import rospy
from std_srvs.srv import Empty
import actionlib
import tf
import numpy
from IPython import embed

from apc_manipulation.msg import ApproachAction, ApproachActionFeedback
import baxter_interface
from baxter_pykdl import baxter_kinematics
import PyKDL
from tf_conversions import posemath as pm

LEFT_ARM = "left_arm"
RIGHT_ARM = "right_arm"
LEFT_ENDPOINT_NAME = "left_gripper"
RIGHT_ENDPOINT_NAME = "right_gripper"
BASE_FRAME = "base"
DEFAULT_FREQ = 50
DEFAULT_FORCE = 10
DEFAULT_VEL = 0.01
DEFAULT_RAMP_VEL_RISE_TIME = 1.0
DEFAULT_TIME = 1.0
DEFAULT_SUCTION_TIMEOUT = 10.0


class ApproachArm:
    """ApproachArm.

    Implements an actionlib server that commands a Baxter end effector
    movement along a pre-defined vector.
    """

    def __init__(self, name):
        """Initialize the class witht the actionlib name."""
        self._action_name = name
        self._action_server = actionlib.SimpleActionServer(self._action_name,
                                                           ApproachAction,
                                                           execute_cb=self.actionlib_callback,
                                                           auto_start=False)
        self._feedback = ApproachActionFeedback()
        self._pause_joint_trajectory_service_left = rospy.ServiceProxy('pause_joint_trajectory_server_left', Empty)
        self._pause_joint_trajectory_service_right = rospy.ServiceProxy('pause_joint_trajectory_server_right', Empty)


        self._kinematics = None
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")

        self._ramp_vel_rise_time = 2.0

        self.init()

    def init(self):
        """Load the controller parameters."""
        self._vel = rospy.get_param("/apc/manipulation/approach/velocity", DEFAULT_VEL)
        self._ramp_vel_rise_time = rospy.get_param('/apc/manipulation/approach/ramp_vel_rise_time', DEFAULT_RAMP_VEL_RISE_TIME)
        self._frequency = rospy.get_param("/apc/manipulation/approach/frequency", DEFAULT_FREQ)
        self._force_limit = rospy.get_param("/apc/manipulation/approach/force_limit", DEFAULT_FORCE)
        self._compensation_time = rospy.get_param("/apc/manipulation/approach/compensation_time", DEFAULT_TIME)
        self._suction_timeout = rospy.get_param("/apc/manipulation/approach/suction_timeout", DEFAULT_SUCTION_TIMEOUT)

        self._action_server.start()

        return True

    def actionlib_callback(self, goal):
        """Implement the controller actionlib callback."""
        if goal.arm == LEFT_ARM:
            arm = self._left_arm
            gripper = baxter_interface.Gripper('left')
            self._kinematics = baxter_kinematics("left")
            pause_joint_trajectory_service = self._pause_joint_trajectory_service_left
        elif goal.arm == RIGHT_ARM:
            arm = self._right_arm
            gripper = baxter_interface.Gripper('right')
            self._kinematics = baxter_kinematics("right")
            pause_joint_trajectory_service = self._pause_joint_trajectory_service_right
        else:
            rospy.logerr("Arm name not valid!")
            self._action_server.set_aborted()
            return

        max_distance = goal.max_distance
        joint_velocities = PyKDL.JntArray(7)
        commanded_velocity = PyKDL.Twist().Zero()
        success = False
        preemption = False

        arm.set_joint_positions(arm.joint_angles())
        pause_joint_trajectory_service()

        # "Gravity compensation"
        # force = None
        begin_time = rospy.get_time()
        # force_collection = numpy.zeros((3, self._compensation_time*self._frequency))
        # i = 0
        # while i < self._compensation_time*self._frequency:
        #     force = self.get_force(arm)
        #     force_collection[:, i] = force
        #     i = i + 1
        #     rospy.sleep(1/self._frequency)
        #
        # force_bias = self.compute_force_bias(force_collection)
        gripper.command_suction(timeout=self._suction_timeout)

        init_position = self.get_arm_position(arm)
        travelled_distance = 0
        vacuum = False
        # numpy.linalg.norm(force - force_bias) < self._force_limit and
        while travelled_distance < max_distance:
            if self._action_server.is_preempt_requested():
                rospy.logwarn('%s: Preempted' % self._action_name)
                success = False
                preemption = True
                gripper.stop()
                self._action_server.set_preempted()
                break

            (trans, rot) = self.get_arm_pose(arm)

            kdl_frame = pm.fromTf((trans, rot))

            movement_vector = kdl_frame.M.UnitZ()

            # calculate ramp velocity profile
            time = rospy.get_time() - begin_time

            if time < self._ramp_vel_rise_time:
                commanded_velocity.vel = self._vel * movement_vector * time/self._ramp_vel_rise_time
            else:
                commanded_velocity.vel = self._vel * movement_vector

            self._kinematics._ik_v_kdl.CartToJnt(
                self._kinematics.joints_to_kdl('positions'),
                commanded_velocity, joint_velocities)

            baxter_joint_velocities = self.kdl_to_joints(arm.joint_names(),
                                                         joint_velocities)

            if baxter_joint_velocities is None:
                success = False
                break

            if vacuum == True:
                success = True
                break

            arm.set_joint_velocities(baxter_joint_velocities)
            # force = self.get_force(arm)
            self._feedback.feedback.force = 0  # numpy.linalg.norm(force - force_bias)
            self._feedback.feedback.distance = travelled_distance
            self._feedback.feedback.vacuum = gripper.vacuum_sensor()
            self._action_server.publish_feedback(self._feedback.feedback)
            rospy.sleep(1/self._frequency)
            vacuum = gripper.vacuum()
            current_position = self.get_arm_position(arm)
            travelled_distance = self.compute_distance(init_position, current_position)

        arm.exit_control_mode()
        if success:
            self._action_server.set_succeeded()
        elif not preemption:
            gripper.stop()
            self._action_server.set_aborted()

        # Set velocity to 0
        baxter_joint_velocities = self.kdl_to_joints(arm.joint_names(),
                                                     PyKDL.JntArray(7))
        arm.set_joint_velocities(baxter_joint_velocities)
        pause_joint_trajectory_service()

    def compute_distance(self, p1, p2):
        """Get the cartesian distance between two positions."""
        p1_np = numpy.array(p1)
        p2_np = numpy.array(p2)

        return numpy.linalg.norm(p2_np - p1_np)

    def get_arm_position(self, arm):
        """Get the arm position in a nice format."""
        pose_dict = arm.endpoint_pose()
        trans = [pose_dict['position'].x, pose_dict['position'].y,
                 pose_dict['position'].z]

        return trans

    def get_arm_orientation(self, arm):
        """Get the arm orientation in a nice format."""
        pose_dict = arm.endpoint_pose()
        rot = [pose_dict['orientation'].x, pose_dict['orientation'].y,
               pose_dict['orientation'].z, pose_dict['orientation'].w]

        return rot

    def get_arm_pose(self, arm):
        """Get the arm endpoint pose in a nice format."""
        trans = self.get_arm_position(arm)
        rot = self.get_arm_orientation(arm)

        return (trans, rot)

    def compute_force_bias(sel, force_collection):
        """Get the average bias for each force collection component."""
        bias = numpy.zeros((3, 1))

        for i in range(0, 3):
            bias[i] = numpy.average(force_collection[i, :])

        return bias.transpose()

    def kdl_to_joints(self, names, values):
        """Map from JntArray to dictionary with joint names."""
        velocities = dict()

        if len(names) != values.rows():
            rospy.logerr("Tried to map %d joint names to %d joint values!" %
                         len(names), values.rows())
            return None

        for i in range(len(names)):
            velocities[names[i]] = values[i]

        return velocities

    def get_force(self, arm):
        """Get force as a numpy array."""
        bad_force = arm.endpoint_effort()["force"]
        good_force = numpy.array([bad_force.x, bad_force.y, bad_force.z])

        return good_force

if __name__ == '__main__':
    rospy.init_node('ApproachNode')
    approach_server = ApproachArm("/apc/manipulation/approach")
    if approach_server.init() is True:
        rospy.loginfo("Started the approach node action server")
        rospy.spin()
