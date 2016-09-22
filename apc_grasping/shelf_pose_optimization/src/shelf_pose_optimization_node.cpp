// STL includes
#include <iostream>
#include <fstream>
#include <string>
// ROS includes
#include "ros/ros.h"
#include "ros/console.h"
#include "visualization_msgs/MarkerArray.h"
// apc stuff
#include "apc_grasping/shelf_pose/ShelfPoseOptimizer.h"
// Eigen
#include <Eigen/Geometry>

void printUsage() {
  ROS_INFO_STREAM(
    "Usage: \n"
    << " 1) shelf_pose_optimization_node <path_to_urdf> <path_to_srdf> <path_to_shelf_mesh> \n"
    << " 2) shelf_pose_optimization_node --paramServer <robot_description> <path_to_shelf_mesh>");
}

int parseArgs(int argc, char *argv[], ros::NodeHandle& nh, apc_grasping::shelf_pose::Arguments &args) {
  if (argc != 4) {
    ROS_ERROR_STREAM("Invalid number of parameters: " << argc);
    printUsage();
    return -1;
  }
  std::string sParamArgument("--paramServer");
  if (sParamArgument.compare(argv[1]) == 0) {
    args.paramsFromServer = true;
    args.paramName.append(argv[2]);
  } else {
    args.paramsFromServer = false;
    std::ifstream ifs;
    ifs.open(argv[1]);
    args.urdf.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    ifs.close();
    ifs.open(argv[2]);
    args.srdf.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    ifs.close();
    if (args.urdf.size() == 0 || args.srdf.size() == 0) {
      ROS_ERROR_STREAM("Could not load robot models.");
      printUsage();
      return -1;
    }
  }
  args.shelfMeshPath.append(argv[3]);
  // TODO read these from somewhere
  args.maxXSamples = 8;
  args.maxYSamples = 8;
  args.maxRotSamples = 3;
  // TODO same here
  args.shelfPoseRange.xmin = -0.2;
  args.shelfPoseRange.xmax = 0.5;
  args.shelfPoseRange.ymin = -0.5;
  args.shelfPoseRange.ymax = 0.5;
  args.shelfPoseRange.rzmin = -0.3; 
  args.shelfPoseRange.rzmax = 0.3;
  args.numEEFRotSamples = 3;
  args.numEEFWidthSamples = 3;
  args.numEEFHeightSamples = 3;
  args.numEEFDepthSamples = 3;
  // ROS_INFO_STREAM("urdf: " << args.urdf << 
  //                 "\nsrdf: " << args.srdf <<
  //                 "\npath_to_shelf_mesh: " << args.shelfMeshPath <<
  //                 "\nparam_name: " << args.paramName);
  return 0;
}

void printPose(const Eigen::Affine3d& pose) {
  ROS_INFO_STREAM("The optimal pose is: " << pose.matrix());
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "Pose_Optimizer");
  apc_grasping::shelf_pose::Arguments args;
  ros::NodeHandle nh;
  int success = parseArgs(argc, argv, nh, args);

  ros::Publisher scenePublisher = nh.advertise<moveit_msgs::PlanningScene>("/shelf_optimization_scene", 1);
  if (!scenePublisher) {
    ROS_ERROR("Could not create planning scene publisher.");
    success = -1;
  }
  ros::Publisher posesPublisher = nh.advertise<visualization_msgs::MarkerArray>("/shelf_bin_sample_poses", 1);
  if (!posesPublisher) {
    ROS_ERROR("Could not create poses publisher.");
    success = -1;
  }
  if (success == 0) {
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
      ros::console::notifyLoggerLevelsChanged();
    }
    // TODO receive result and print it
    apc_grasping::shelf_pose::ShelfPoseOptimizer optimizer;
    optimizer.setScenePublisher(scenePublisher);
    optimizer.setPosesPublisher(posesPublisher);
    bool initSuccess = optimizer.initialize(args);
    if (!initSuccess) {
      return -1;
    }
    Eigen::Affine3d pose = optimizer.run();
    printPose(pose);
  }
  return success;
}