#include <ros/ros.h>
#include "feature_segmentation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_feature_segmentation");
  ros::NodeHandle nh;

  FeatureSegmentation node(nh);

  if (!node.start()) {
    ROS_ERROR("feature_segmentation::Node not ready, cannot proceed.");
    return 1;
  }

  ros::spin();
  return 0;
}
