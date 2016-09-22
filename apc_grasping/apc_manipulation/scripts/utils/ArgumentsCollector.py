#!/usr/bin/env python
import rospy
import tf_conversions
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState


class ArgumentsCollector(object):
    """
        This class acquires the arguments for the specified BT Action.

        @author: Joshua Haustein (haustein@kth.se)
    """

    def __init__(self, namespace, listOfArgumentNames):
        """ Creates a new argument collector.
            @param namespace - the namespace (i.e. prefix) that all arguments share
            @param listOfArgumentNames - a list of the names of the arguments to acquire
        """
        self.namespace = namespace
        self.listOfArgumentNames = listOfArgumentNames

    def getArguments(self, optional=False):
        """ @return a map from argument name to argument value
            @throws KeyError if any argument is not available if optional==False.
        """
        if not rospy.has_param(self.namespace):
            if not optional:
                raise KeyError('Could not retrieve any arguments. There is nothing on the parameter server for this namespace: ' + self.namespace)
            return {}
        arguments = rospy.get_param(self.namespace)
        missingArguments = []
        for argument in self.listOfArgumentNames:
            if argument not in arguments:
                missingArguments.append(argument)
        if not optional and len(missingArguments) > 0:
            rospy.logerr('Could not retrieve all arguments for %s from parameter server. ' +
                         'Missing arguments are: %l') % (self.namespace, missingArguments)
            raise KeyError('The arguments %l are missing.') % missingArguments
        return arguments


def mapToPose(poseMap):
    pose = PoseStamped()
    header = poseMap['header']
    pose.header.frame_id = header['frame_id']
    pose.header.seq = header['seq']
    stamp = header['stamp']
    pose.header.stamp.secs = stamp['secs']
    pose.header.stamp.nsecs = stamp['nsecs']
    pose.header.frame_id = header['frame_id']
    position = poseMap['pose']['position']
    pose.pose.position.x = position['x']
    pose.pose.position.y = position['y']
    pose.pose.position.z = position['z']
    orientation = poseMap['pose']['orientation']
    pose.pose.orientation.x = orientation['x']
    pose.pose.orientation.y = orientation['y']
    pose.pose.orientation.z = orientation['z']
    pose.pose.orientation.w = orientation['w']
    return pose


def poseToMap(pose):
    stampedPoseMap = {}
    headerMap = {}
    headerMap['frame_id'] = pose.header.frame_id
    headerMap['seq'] = pose.header.seq
    stampMap = {}
    stampMap['secs'] = pose.header.stamp.secs
    stampMap['nsecs'] = pose.header.stamp.nsecs
    headerMap['stamp'] = stampMap
    stampedPoseMap['header'] = headerMap
    poseMap = {}
    positionMap = {}
    positionMap['x'] = pose.pose.position.x
    positionMap['y'] = pose.pose.position.y
    positionMap['z'] = pose.pose.position.z
    orientationMap = {}
    orientationMap['x'] = pose.pose.orientation.x
    orientationMap['y'] = pose.pose.orientation.y
    orientationMap['z'] = pose.pose.orientation.z
    orientationMap['w'] = pose.pose.orientation.w
    poseMap['position'] = positionMap
    poseMap['orientation'] = orientationMap
    stampedPoseMap['pose'] = poseMap
    return stampedPoseMap


def createROSPose(position, rotation, frame_id='/base'):
    """ Creates a stamped pose from a list position and a list rotation.
        @param position - [x, y, z]
        @param rotation - [x, y, z, w]
        @param frame_id - the frame
    """
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    stampedPose = PoseStamped(header=header)
    stampedPose.pose.position.x = position[0]
    stampedPose.pose.position.y = position[1]
    stampedPose.pose.position.z = position[2]
    stampedPose.pose.orientation.x = rotation[0]
    stampedPose.pose.orientation.y = rotation[1]
    stampedPose.pose.orientation.z = rotation[2]
    stampedPose.pose.orientation.w = rotation[3]
    return stampedPose


def stampedPoseToList(rosPose):
    return poseToList(rosPose.pose)


def poseToList(rosPose, euler=False):
    listPose = [rosPose.position.x, rosPose.position.y, rosPose.position.z,
                rosPose.orientation.x, rosPose.orientation.y, rosPose.orientation.z,
                rosPose.orientation.w]
    if euler:
        return listPoseQuatTolistPoseEuler(listPose)
    return listPose


def listPoseQuatTolistPoseEuler(listPose):
    eulerList = list(listPose[:3])
    eulerList.extend(tf_conversions.transformations.euler_from_quaternion(listPose[3:7]))
    return eulerList


def dictToJointState(dictConfig):
    header = Header(stamp=rospy.Time.now())
    names = []
    positions = []
    for (k, v) in dictConfig.iteritems():
        names.append(k)
        positions.append(v)

    return JointState(header=header, name=names, position=positions)

def dictFromJointState(names, positions):
    dictConfig = {}
    for i in range(len(names)):
        dictConfig[names[i]] = positions[i]
    return dictConfig 

def makeStamped(pose, frame_id='/base'):
    header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
    return PoseStamped(header=header, pose=pose)


class BoundingBox:
    def __init__(self):
        self.extents = []
        for r in range(3):
            self.extents.append([0.0, 0.0])

    def addPosition(self, x, y, z):
        self.extents[0][0] = min(self.extents[0][0], x)
        self.extents[0][1] = max(self.extents[0][1], x)
        self.extents[1][0] = min(self.extents[1][0], y)
        self.extents[1][1] = max(self.extents[1][1], y)
        self.extents[2][0] = min(self.extents[2][0], z)
        self.extents[2][1] = max(self.extents[2][1], z)

    def getArea(self):
        edges = map(lambda x: x[1] - x[0], self.extents[:3])
        nonzero_edges = filter(lambda x: abs(x) > 1e-5, edges)
        if len(nonzero_edges) > 1:
            return reduce(lambda x, y: x * y, nonzero_edges)
        return 0.0
