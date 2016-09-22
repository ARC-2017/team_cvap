// APC includes
#include "apc_grasping/shelf_pose/ShelfPoseOptimizer.h"
// boost includes
#include <boost/make_shared.hpp>
#include <boost/variant/get.hpp>
// stl includes
#include <fstream>
// #include <functional>
// moveit includes
#include "moveit/robot_model/joint_model_group.h"
#include "moveit/robot_state/robot_state.h"
// #include "moveit/robot_state/robot_state.h"
// #include "moveit/kinematics_metrics/kinematics_metrics.h"
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_messages.h"
#include "geometric_shapes/shape_operations.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "eigen_conversions/eigen_msg.h"

using namespace apc_grasping::shelf_pose;

ShelfPoseOptimizer::ShelfPoseOptimizer():
   _scenePublisherAvailable(false),
   _posesPublisherAvailable(false) { }
ShelfPoseOptimizer::~ShelfPoseOptimizer() { }

bool ShelfPoseOptimizer::initialize(const Arguments& args) {
  _args = args;
  robot_model_loader::RobotModelLoader::Options robotLoaderOptions;
  if (args.paramsFromServer) {
    // urdfModelLoaded = urdfModel->initParam(args.paramName);
    robotLoaderOptions.robot_description_ = args.paramName;
  } else {
    // urdfModelLoaded = urdfModel->initFile(args.urdfPath);
    robotLoaderOptions.robot_description_ = "";
    robotLoaderOptions.urdf_string_ = args.urdf;
    robotLoaderOptions.srdf_string_ = args.srdf;
  }
  robotLoaderOptions.load_kinematics_solvers_ = true;
  _robotLoader = boost::make_shared<robot_model_loader::RobotModelLoader>(robotLoaderOptions);
  _robotModel = _robotLoader->getModel();
  _planningScene = boost::make_shared<planning_scene::PlanningScene>(_robotModel);

  moveit::core::RobotState& robotState = _planningScene->getCurrentStateNonConst();
 _armRestingConfigurationLeft = {0.6, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};
 _armRestingConfigurationRight = {-0.6, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};
 _groupNameRight = "right_arm";
 _groupNameLeft = "left_arm";
 robotState.setJointGroupPositions(_groupNameRight, _armRestingConfigurationRight);
 robotState.setJointGroupPositions(_groupNameLeft, _armRestingConfigurationLeft);

  // Create shelf
  moveit_msgs::CollisionObject shelfCol;
  shelfCol.header.frame_id = "/base";
  shelfCol.id = "shelf";
  // Load mesh
  shape_msgs::Mesh meshRosMessage;
  shapes::ShapeMsg meshMessage = meshRosMessage;
  
  bool couldLoadShape = loadMesh(args.shelfMeshPath, meshMessage);
  if (!couldLoadShape) {
    ROS_ERROR("Could not load shelf mesh!");
    return false;
  }
  shelfCol.meshes.push_back(boost::get<shape_msgs::Mesh>(meshMessage));
  // set a dummy pose
  geometry_msgs::Pose shelf_pose;
  // shelf_pose.orientation.x = 0.7071;
  // shelf_pose.orientation.y = 0.0;
  // shelf_pose.orientation.z = 0.0;
  // shelf_pose.orientation.w = 0.7071;
  shelf_pose.orientation.x = 0.5;
  shelf_pose.orientation.y = -0.5;
  shelf_pose.orientation.z = -0.5;
  shelf_pose.orientation.w = 0.5;
  shelf_pose.position.x = 0.0;
  shelf_pose.position.y = 0.0;
  shelf_pose.position.z = -0.93;
  shelfCol.mesh_poses.push_back(shelf_pose);
  // set operation type of shelfCol.... man, this is tedious....
  shelfCol.operation = shelfCol.ADD;
  // Finally, we can add the shelf to the planning scene (yay see the number of lines of code? amazing!)
  _planningScene->processCollisionObjectMsg(shelfCol);
  _shelfBasePose = _planningScene->getFrameTransform("shelf");

  computeSamplePoses(_eefSamplePoses);
  publishScene();  
  _firstPosePublishing = true;
  _initialized = true;
  publishPoses();

  ROS_INFO_STREAM("Computed " << _eefSamplePoses.size() << " eef poses. Ready for optimization.");
  // TODO catch failure and return false
  return _initialized;
}

Eigen::Affine3d ShelfPoseOptimizer::run() {
  Eigen::Affine3d bestShelfPose;
  if (!_initialized) {
    return bestShelfPose;
  }
  // Eigen::Affine3d shelfBasePose = _planningScene->getFrameTransform("shelf");
  
  // optimize
  float maxScore = -1.0;
  for (unsigned int xSample = 1; xSample < _args.maxXSamples + 1; ++xSample) {
    double x = _args.shelfPoseRange.xmin + xSample * (_args.shelfPoseRange.xmax - _args.shelfPoseRange.xmin) / float(_args.maxXSamples +1);
    for (unsigned int ySample = 1; ySample < _args.maxYSamples + 1; ++ySample) {
      double y = _args.shelfPoseRange.ymin + ySample * (_args.shelfPoseRange.ymax - _args.shelfPoseRange.ymin) / float(_args.maxYSamples +1);
      for (unsigned int rotSample = 1; rotSample < _args.maxRotSamples + 1; ++rotSample) {
        if (!ros::ok()) return bestShelfPose;
        double rot = _args.shelfPoseRange.rzmin + rotSample * (_args.shelfPoseRange.rzmax - _args.shelfPoseRange.rzmin) / float(_args.maxRotSamples +1);
        Eigen::Affine3d shelfPose = Eigen::Translation<double, 3>(Eigen::Vector3d(x,y,0.0)) * Eigen::AngleAxisd(rot, Eigen::Vector3d::UnitZ());
        shelfPose = shelfPose * _shelfBasePose;
        float score = evaluateBasePose(shelfPose);
        // ROS_INFO_STREAM("The score for this pose is " << score);  
        if (score > maxScore) {
          ROS_INFO_STREAM("Found a new best pose with score: " << score);
          ROS_INFO_STREAM("Values are (relative to shelf base pose) x: " << x << ", y: " << y << ", rot: " << rot);
          publishScene();
          publishPoses();
          maxScore = score;
          bestShelfPose = shelfPose;
        }
      }
    }
  }
  
  return bestShelfPose;
}

void ShelfPoseOptimizer::setScenePublisher(std::string name) {
  ros::NodeHandle nh;
  ros::Publisher scenePublisher = nh.advertise<moveit_msgs::PlanningScene>(name, 1);
  setScenePublisher(scenePublisher);
}

void ShelfPoseOptimizer::setPosesPublisher(std::string name) {
  ros::NodeHandle nh;
  ros::Publisher scenePublisher = nh.advertise<visualization_msgs::MarkerArray>(name, 1);
  setPosesPublisher(scenePublisher);
}

void ShelfPoseOptimizer::setScenePublisher(ros::Publisher& publisher) {
  _scenePublisher = publisher;
  _scenePublisherAvailable = true;
}

void ShelfPoseOptimizer::setPosesPublisher(ros::Publisher& publisher) {
  _posesPublisher = publisher;
  _posesPublisherAvailable = true;
}

float ShelfPoseOptimizer::evaluateShelfPose(float x, float y, float theta, bool publishInitialScene, bool publishRobotStates) {
  Eigen::Affine3d shelfPose = Eigen::Translation<double, 3>(Eigen::Vector3d(x,y,0.0)) * Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) * _shelfBasePose;  
  return evaluateBasePose(shelfPose, publishInitialScene, publishRobotStates);
}

float ShelfPoseOptimizer::evaluateBasePose(const Eigen::Affine3d& shelfPose, bool publishInitialScene, bool publishRobotStates) {
  // first move the shelf
  moveit_msgs::CollisionObject shelfCol;
  shelfCol.header.frame_id = "/base";
  shelfCol.id = "shelf";
  geometry_msgs::Pose rosPose;
  tf::poseEigenToMsg(shelfPose, rosPose);
  shelfCol.mesh_poses.push_back(rosPose);
  shelfCol.operation = shelfCol.MOVE;
  _planningScene->processCollisionObjectMsg(shelfCol);
  if (publishInitialScene) {
    publishScene();
    publishPoses();
  }
  
  ROS_INFO_STREAM("Args are: " << _args.numEEFWidthSamples << " " << _args.numEEFHeightSamples
                  << " " << _args.numEEFDepthSamples << " " << _args.numEEFRotSamples);

  unsigned int numReachable = 0;
  unsigned int numIksFound = 0;
  // iterate over eef poses in samplePoses
  for (unsigned int i = 0; i < _eefSamplePoses.size(); ++i) {
    Eigen::Affine3d globalPose = shelfPose * _eefSamplePoses.at(i);
    // ROS_INFO_STREAM("Testing eef pose " << i);
    bool hasIk = checkPoseForIk(_groupNameLeft, globalPose, numIksFound, numReachable);
    if (publishRobotStates && hasIk) {
      publishScene();
      ros::Duration(0.1).sleep();
    }
    hasIk = checkPoseForIk(_groupNameRight, globalPose, numIksFound, numReachable);
    if (publishRobotStates && hasIk) {
      publishScene();
      ros::Duration(0.1).sleep();
    }
    // ROS_INFO_STREAM("Found " << numIksFound << " iks so far.");
    // ROS_INFO_STREAM("Found " << numReachable << " collision-free iks so far.");
  }
  
  // TODO combine all results to single number
  // TODO return this number
  return float(numReachable) / float(2 * _eefSamplePoses.size());
}

bool isStateValid(planning_scene::PlanningScenePtr planningScene, robot_state::RobotState* rs, const moveit::core::JointModelGroup* jg, const double* jgv) {
  rs->setJointGroupPositions(jg, jgv);
  collision_detection::CollisionRequest req;
  req.contacts = false; //true;
  req.group_name = jg->getName();
  collision_detection::CollisionResult res;
  planningScene->checkCollisionUnpadded(req, res, *rs);
  return !res.collision && rs->satisfiesBounds();
}

bool ShelfPoseOptimizer::checkPoseForIk(const std::string& armGroupName,
                                        const Eigen::Affine3d& globalPose, unsigned int& numIksFound, unsigned int& numReachable) {
  
  const moveit::core::JointModelGroup* armGroup = _robotModel->getJointModelGroup(armGroupName);
  moveit::core::RobotState& robotState = _planningScene->getCurrentStateNonConst();
  robotState.setJointGroupPositions(_groupNameLeft, _armRestingConfigurationLeft);
  robotState.setJointGroupPositions(_groupNameRight, _armRestingConfigurationRight);
  moveit::core::GroupStateValidityCallbackFn validityFn = boost::bind(isStateValid, _planningScene, _1, _2, _3);
  bool hasIk = robotState.setFromIK(armGroup, globalPose, 3, 0.001, validityFn);
  numReachable += hasIk;
  // if (hasIk) {
    // publishScene();
    // ros::Duration(0.2).sleep();
    // ++numIksFound;
    // ++numReachable;
    // collision_detection::CollisionRequest req;
    // req.contacts = false; //true;
    // req.group_name = armGroupName;
    // collision_detection::CollisionResult res;
    // _planningScene->checkCollisionUnpadded(req, res);
    // if (res.collision) ROS_ERROR("HAVE A COLLISION!");
    // numReachable += !res.collision;

    // for (auto& element : res.contacts) {
    //   std::cout << "Contact between " << (element.first).first << " and " << (element.first).second << std::endl;
    // }
  // }
  return hasIk;
}

void ShelfPoseOptimizer::publishScene() {
  if (not _scenePublisherAvailable) {
    return;
  }
  // ROS_INFO("Publishing scene.");
  moveit_msgs::PlanningScene planningSceneMsg;
  _planningScene->getPlanningSceneMsg(planningSceneMsg);
  _scenePublisher.publish(planningSceneMsg);
}

void ShelfPoseOptimizer::publishPoses() {
  if (not _posesPublisherAvailable) {
    return;
  }
  // ROS_INFO("Publishing poses.");
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base"; 
  marker.ns = "shelf_bin_pose_reachability";
  if (_firstPosePublishing) {
    marker.header.stamp = ros::Time::now();
    marker.action =  3;//visualization_msgs::Marker::DELETEALL;
    markerArray.markers.push_back(marker);
    _posesPublisher.publish(markerArray);
    markerArray.markers.clear();
  }

  Eigen::Affine3d shelfPose = _planningScene->getFrameTransform("shelf");
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  for (size_t i = 0; i < _eefSamplePoses.size(); ++i) {
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    Eigen::Affine3d globalPose = shelfPose * _eefSamplePoses.at(i);
    tf::poseEigenToMsg(globalPose, marker.pose);
    // TODO choose color based on reachability
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    markerArray.markers.push_back(marker);
  }
  // std::cout << "publishing:" << std::endl << markerArray << std::endl;
  _posesPublisher.publish(markerArray);
}

bool ShelfPoseOptimizer::loadMesh(std::string filePath, shapes::ShapeMsg& meshMessage) {
  std::ifstream ifs(filePath.c_str());
  // std::cout << "opening "  << filePath << std::endl;
  bool couldLoadShape = false;
  if (ifs) {
    // get length of the file:
    ifs.seekg(0, ifs.end);
    int length = ifs.tellg();
    ifs.seekg(0, ifs.beg);
    char* buffer = new char[length];
    // read data
    ifs.read(buffer, length);
    ifs.close();
    // std::cout.write (buffer,length);
    // std::cout << "\nopened file, l:" << length << std::endl;
    shapes::Mesh* pShelfMesh = shapes::createMeshFromBinary(buffer, length, filePath);
    delete buffer;
    if (pShelfMesh) {
      // std::cout << "num_vertices: " << pShelfMesh->vertex_count << std::endl;
      // std::cout << "triangle_count: " << pShelfMesh->triangle_count << std::endl;
      couldLoadShape = shapes::constructMsgFromShape(pShelfMesh, meshMessage);
      delete pShelfMesh;
    } 
  }
  return couldLoadShape;
}

void ShelfPoseOptimizer::computeSamplePoses(EigenSTL::vector_Affine3d& poses) {
  poses.clear();
  // compute base poses of bins first
  // this is the base pose of bin 0,0 (J)
  Eigen::Affine3d basePose;
  basePose.setIdentity();
  basePose(0,0) = -1.0;
  basePose(2,2) = -1.0;
  basePose(0,3) = -0.42;
  basePose(1,3) = 0.8; // 0.825
  basePose(2,3) = 0.46; // 0.42 
  std::vector<float> columnWidths = {SIDE_COL_WIDTH, CENTER_COL_WIDTH, SIDE_COL_WIDTH};
  std::vector<float> rowHeights = {BOTTOM_ROW_HEIGHT, MIDDLE_ROW_HEIGHT, MIDDLE_ROW_HEIGHT, TOP_ROW_HEIGHT};
  std::vector< EigenSTL::vector_Affine3d > binPoses;
  computeBinPoses(basePose, columnWidths, rowHeights, binPoses);

  // Now sample each bin
  for (unsigned int c = 0; c < NUM_COLUMNS; ++c) {
    for (unsigned int r = 0; r < NUM_ROWS; ++r) {
      EigenSTL::vector_Affine3d innerBinPoses;
      computePosesWithinBin(binPoses.at(c).at(r), innerBinPoses, columnWidths[c], rowHeights[r],
                            BIN_DEPTH, -0.78, 0.78, 0.0, _args.numEEFWidthSamples,
                            _args.numEEFHeightSamples, _args.numEEFDepthSamples,
                            _args.numEEFRotSamples);
      poses.insert(poses.end(), innerBinPoses.begin(), innerBinPoses.end());
    } 
  } 
}

void ShelfPoseOptimizer::computeBinPoses(const Eigen::Affine3d& basePose, const std::vector<float>& columnWidths,
                                         const std::vector<float>& rowHeights, std::vector< EigenSTL::vector_Affine3d >& binPoses) {
  binPoses.clear();
  float x = 0.0;
  for (unsigned int c = 0; c < NUM_COLUMNS; ++c) {
    binPoses.push_back(EigenSTL::vector_Affine3d());
    float y = 0.0;
    for (unsigned int r = 0; r < NUM_ROWS; ++r) {
      Eigen::Affine3d pose(basePose);
      pose(0, 3) += x;
      pose(1, 3) += y;
      y += rowHeights[r];
      binPoses[c].push_back(pose);
    }
    x += columnWidths[c];
  } 
}

void ShelfPoseOptimizer::computePosesWithinBin(const Eigen::Affine3d& binPose, EigenSTL::vector_Affine3d& poses, 
                                     float binWidth, float binHeight, float binDepth,
                                     float rotMin, float rotMax, 
                                     float padding, unsigned int numWidthSamples, 
                                     unsigned int numHeightSamples, unsigned int numDepthSamples,
                                     unsigned int numRotSamples) {
  poses.clear();
  Eigen::Affine3d pose(binPose);
  float rotStep = (rotMax - rotMin) / float(numRotSamples+1);
  for (unsigned int wSample = 1; wSample < numWidthSamples + 1; ++wSample) {
    pose(0, 3) = binPose(0, 3) + wSample * (binWidth - 2.0 * padding) / float(numWidthSamples + 1) + padding;
    for (unsigned int hSample = 1; hSample < numHeightSamples + 1; ++hSample) {
      pose(1, 3) = binPose(1, 3) + hSample * (binHeight - 2.0 * padding) / float(numHeightSamples + 1) + padding;
      for (unsigned int dSample = 1; dSample < numDepthSamples + 1; ++dSample) {
        pose(2, 3) = binPose(2, 3) - dSample * (binDepth - 2.0 * padding) / float(numDepthSamples + 1) - padding;
        for (unsigned int rxSample = 1; rxSample < numRotSamples + 1; ++rxSample) {
          Eigen::AngleAxisd rxMatrix(rxSample * rotStep + rotMin, Eigen::Vector3d::UnitX());
          for (unsigned int rySample = 1; rySample < numRotSamples + 1; ++rySample) {
            Eigen::AngleAxisd ryMatrix(rySample * rotStep + rotMin, Eigen::Vector3d::UnitY());
            for (unsigned int rzSample = 1; rzSample < numRotSamples + 1; ++rzSample) {
              Eigen::AngleAxisd rzMatrix(rzSample * rotStep + rotMin, Eigen::Vector3d::UnitZ());
              poses.push_back(pose * rxMatrix * ryMatrix * rzMatrix);
            }
          }
        }
      }
    }
  }
}
