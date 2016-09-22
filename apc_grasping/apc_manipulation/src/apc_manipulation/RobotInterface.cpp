#include "apc_manipulation/RobotInterface.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <fstream>
#include <stdexcept>

using namespace apc;
RobotInterface::RobotInterface() {
  std::string leftArm("left_arm");
  std::string rightArm("right_arm");
  _leftArmGroup =
      std::make_shared<moveit::planning_interface::MoveGroup>(leftArm);
  _rightArmGroup =
      std::make_shared<moveit::planning_interface::MoveGroup>(rightArm);
  _leftGripper = std::make_shared<BaxterGripper>(leftArm);
  _rightGripper = std::make_shared<BaxterGripper>(rightArm);
}

RobotInterface::~RobotInterface() {}

bool RobotInterface::acquireLock() { return _mutex.try_lock(); }

void RobotInterface::releaseLock() { _mutex.unlock(); }

bool RobotInterface::resetPlanningScene() {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  std::vector<std::string> allObjects =
      _planningSceneInterface.getKnownObjectNames();
  ROS_INFO_STREAM("Removing all objects from the planning scene!");
  for (std::string name : allObjects) {
    ROS_INFO_STREAM("removing: " << name);
  }
  _planningSceneInterface.removeCollisionObjects(allObjects);

  bool success = false;
  bool hasShelfTransform = false;
  bool hasShelfMesh = false;

  geometry_msgs::PoseStamped shelfPose;
  geometry_msgs::PoseStamped originPose;
  std::string shelfMeshPath;
  std::string objName("shelf");
  originPose.header.frame_id = objName;
  std::string baseFrame("base");
  std::string shelfPathParameterName("/apc/shelf_mesh_path");
  while (_nodeHandle.ok() && !success) {
    hasShelfTransform = getPose(objName, shelfPose);
    if (hasShelfTransform) {
      success = addObject(objName, shelfPose);
      if (!success) {
        ROS_ERROR_STREAM("Could not load mesh model for shelf.");
        ros::Duration(1.0).sleep();
      }
    } else {
      ROS_WARN("Could not receive shelf pose.");
      ros::Duration(1.0).sleep();
    }
  }
  return success;
}

void RobotInterface::addObjectsToPlanningScene(
    const std::vector<std::string> &objectNames) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  for (std::string objName : objectNames) {
    geometry_msgs::PoseStamped objPose;
    bool hasPose = getObjectPose(objName, objPose);
    bool success = false;
    if (hasPose) {
      success = addObject(objName, objPose);
    }
    if (!success) {
      ROS_ERROR_STREAM("Failed to add object " << objName
                                               << " to planning scene.");
    }
  }
}

void RobotInterface::attachObject(const std::string &objectName,
                                  const std::string &armName) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
}

void RobotInterface::releaseObject(const std::string &objectName) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
}

Configuration
RobotInterface::getNamedConfiguration(const std::string &name,
                                      const std::string &arm) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  Configuration config;
  // TODO we should read these from a yaml file
  if (arm == "left_arm") {
    if (name == "HOME") {
      config.emplace("left_e0", -0.609338);
      config.emplace("left_e1", 2.115557);
      config.emplace("left_s0", 1.649642);
      config.emplace("left_s1", -0.946383);
      config.emplace("left_w0", -1.097683);
      config.emplace("left_w1", 1.448012);
      config.emplace("left_w2", -2.155289);
    }
  } else if (arm == "right_arm") {
    config.emplace("right_e0", 0.490008);
    config.emplace("right_e1", 2.001850);
    config.emplace("right_s0", -1.504572);
    config.emplace("right_s1", -0.208330);
    config.emplace("right_w0", 0.949876);
    config.emplace("right_w1", 1.064278);
    config.emplace("right_w2", -1.442308);
  }
  return config;
}

void RobotInterface::removeObjectsFromPlanningScene(
    const std::vector<std::string> &objectNames) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  _planningSceneInterface.removeCollisionObjects(objectNames);
}

MoveResult RobotInterface::moveToPosition(
    const Eigen::Vector3d &position, const std::string &arm,
    moveit::planning_interface::MoveGroup::Plan &plan, bool execute) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
}

MoveResult RobotInterface::moveToPosition(
    const geometry_msgs::PointStamped &position, const std::string &arm,
    moveit::planning_interface::MoveGroup::Plan &plan, bool execute) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  moveHeadAside(arm);
  MoveGroupPtr moveGroup = getMoveGroup(arm);
  moveGroup->setPositionTarget(position.point.x, position.point.y,
                               position.point.z);
  bool success = moveGroup->plan(plan);
  if (!success) {
    ROS_ERROR("Could not find a path to position.");
    return PlanFail;
  }
  if (!execute) {
    return Success;
  }
  success = moveGroup->execute(plan);
  if (success) {
    return Success;
  }
  return ExecuteFail;
}

MoveResult
RobotInterface::moveToPose(const Eigen::Affine3d &pose, const std::string &arm,
                           moveit::planning_interface::MoveGroup::Plan &plan,
                           bool execute) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
}

MoveResult RobotInterface::moveToPose(
    const geometry_msgs::PoseStamped &pose, const std::string &arm,
    moveit::planning_interface::MoveGroup::Plan &plan, bool execute) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  moveHeadAside(arm);
  // TODO do we need sanity checks on the pose here?
  MoveGroupPtr moveGroup = getMoveGroup(arm);
  moveGroup->setPoseTarget(pose);
  bool success = moveGroup->plan(plan);
  if (!success) {
    ROS_ERROR("Could not find a path to pose.");
    return PlanFail;
  }
  if (!execute) {
    return Success;
  }
  success = moveGroup->execute(plan);
  if (success) {
    return Success;
  }
  return ExecuteFail;
}

MoveResult RobotInterface::moveToConfiguration(
    const Configuration &config, const std::string &arm,
    moveit::planning_interface::MoveGroup::Plan &plan, bool execute) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  moveHeadAside(arm);
  if (config.size() != 7) {
    std::string errorMsg(
        "The given configuration must contain exactly 7 joint values.");
    throw std::invalid_argument(errorMsg);
  }
  MoveGroupPtr moveGroup = getMoveGroup(arm);
  moveGroup->setJointValueTarget(config);
  bool success = moveGroup->plan(plan);
  if (!success) {
    ROS_ERROR("Could not find a path to configuration.");
    return PlanFail;
  }
  if (!execute) {
    return Success;
  }
  success = moveGroup->execute(plan);
  if (success) {
    return Success;
  }
  return ExecuteFail;
}

void RobotInterface::moveHeadAside(const std::string &arm) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  // TODO
}

MoveGroupPtr RobotInterface::getMoveGroup(const std::string &armName,
                                          bool clearGroup) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  MoveGroupPtr returnGroup;
  if (armName == "left_arm") {
    returnGroup = _leftArmGroup;
  } else if (armName == "right_arm") {
    returnGroup = _rightArmGroup;
  } else {
    std::string errorMsg("armName must be either left_arm or right_arm.");
    throw std::invalid_argument(errorMsg);
  }
  if (clearGroup) {
    returnGroup->clearPoseTargets();
  }
  return returnGroup;
}

BaxterGripperPtr RobotInterface::getGripper(const std::string &armName) {
  if (armName == "left_arm") {
    return _leftGripper;
  } else if (armName == "right_arm") {
    return _rightGripper;
  }
  std::string errorMsg("armName must be either left_arm or right_arm.");
  throw std::invalid_argument(errorMsg);
}

std::string RobotInterface::getGripperName(const std::string &armName) const {
  if (armName == "left_arm") {
    return "left";
  } else if (armName == "right_arm") {
    return "right";
  }
  std::string errorMsg("armName must be either left_arm or right_arm.");
  throw std::invalid_argument(errorMsg);
}

std::string
RobotInterface::getGripperFrameName(const std::string &armName) const {
  return getGripperName(armName) + "_gripper";
}

std::string
RobotInterface::getObjectFrameName(const std::string &objectName) const {
  return objectName + "_final";
}

bool RobotInterface::getObjectPose(const std::string &objectName,
                                   geometry_msgs::PoseStamped &outPose,
                                   std::string base_frame) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  std::string objectFrameName = getObjectFrameName(objectName);
  return getPose(objectFrameName, outPose, base_frame);
}

bool RobotInterface::getPose(const std::string &frame_id,
                             geometry_msgs::PoseStamped &outPose,
                             std::string base_frame) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  geometry_msgs::PoseStamped inPose;
  inPose.header.frame_id = frame_id;
  inPose.header.stamp = ros::Time::now();
  return transformPose(inPose, outPose, base_frame);
}

Eigen::Affine3d
RobotInterface::getEigenTransform(const std::string &frame_id,
                                  std::string base_frame) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
}

bool RobotInterface::transformPose(const geometry_msgs::PoseStamped &inPose,
                                   geometry_msgs::PoseStamped &outPose,
                                   const std::string &targetFrame) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  try {
    _tfListener.transformPose(targetFrame, inPose, outPose);
  } catch (tf::TransformException ex) {
    ROS_WARN_STREAM("Could not transform pose from " << inPose.header.frame_id
                                                     << " to " << targetFrame);
    return false;
  }
  return true;
}

Eigen::Affine3d
RobotInterface::transformEigenPose(const Eigen::Affine3d &pose,
                                   const std::string &sourceFrame,
                                   const std::string &targetFrame) const {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
}

bool RobotInterface::addObject(const std::string &objectName,
                               const geometry_msgs::PoseStamped &objectPose) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  std::string meshPath = getMeshPath(objectName);
  if (meshPath.length() == 0) {
    return false;
  }
  // Create collision object
  moveit_msgs::CollisionObject colObject;
  colObject.header = objectPose.header;
  colObject.id = objectName;
  // Load mesh
  shape_msgs::Mesh meshRosMessage;
  shapes::ShapeMsg meshMessage = meshRosMessage;

  bool couldLoadMesh = loadMesh(meshPath, meshMessage);
  if (!couldLoadMesh) {
    ROS_ERROR_STREAM("Could not load mesh for object " << objectName);
    return false;
  }
  colObject.meshes.push_back(boost::get<shape_msgs::Mesh>(meshMessage));
  colObject.mesh_poses.push_back(objectPose.pose);
  colObject.operation = colObject.ADD;
  std::vector<moveit_msgs::CollisionObject> v;
  v.push_back(colObject);
  _planningSceneInterface.addCollisionObjects(v);
  return true;
}

void RobotInterface::moveObject(const std::string &objectName,
                                const geometry_msgs::PoseStamped &objectPose) {
  std::lock_guard<std::recursive_mutex> lock(_mutex);
  moveit_msgs::CollisionObject colObj;
  colObj.operation = colObj.MOVE;
  colObj.id = objectName;
  colObj.mesh_poses.push_back(objectPose.pose);
  colObj.header.stamp = objectPose.header.stamp;
  colObj.header.frame_id = objectPose.header.frame_id;
  std::vector<moveit_msgs::CollisionObject> v;
  v.push_back(colObj);
  _planningSceneInterface.addCollisionObjects(v);
}

std::string RobotInterface::getMeshPath(const std::string &objName) const {
  std::string path;
  std::string paramName = "/apc/" + objName + "_mesh_path";
  if (!_nodeHandle.getParam(paramName, path)) {
    ROS_ERROR_STREAM("Could not retrieve mesh path for object " << objName);
    return std::string();
  }
  return path;
}

bool RobotInterface::loadMesh(const std::string &filePath,
                              shapes::ShapeMsg &meshMessage) const {
  std::ifstream ifs(filePath.c_str());
  bool couldLoadShape = false;
  if (ifs) {
    // get length of the file:
    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    char *buffer = new char[length];
    // read data
    ifs.read(buffer, length);
    ifs.close();
    shapes::Mesh *pShelfMesh =
        shapes::createMeshFromBinary(buffer, length, filePath);
    delete buffer;
    if (pShelfMesh) {
      couldLoadShape = shapes::constructMsgFromShape(pShelfMesh, meshMessage);
      delete pShelfMesh;
    }
  }
  return couldLoadShape;
}
