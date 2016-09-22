#include <ros/ros.h>
#include "pc_segmentation.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "perception_pc_segmentation");
  ros::NodeHandle nh;

  PCSegmentation node(nh);

  if (!node.start()) {
    ROS_ERROR("pc_segmentation::Node not ready, cannot proceed.");
    return 1;
  }

  ros::spin();
  return 0;
}
