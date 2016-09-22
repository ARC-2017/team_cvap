#include "ros/ros.h"
#include "ros/console.h"
#include "apc_manipulation/MoveArm.h"
#include "apc_manipulation/PickObject.h"
#include "apc_manipulation/PlaceObject.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "manipulation_node");
  apc::MoveArm moveArm("/apc/manipulation/move_arm");
  apc::PickObject pickObject("/apc/manipulation/pick_object");
  apc::PlaceObject placeObject("/apc/manipulation/place_object");
  ros::spin();
  return 0;
}
