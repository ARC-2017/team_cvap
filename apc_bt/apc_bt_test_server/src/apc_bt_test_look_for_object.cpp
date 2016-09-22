#include <apc_bt_test_server/apc_bt_test_look_for_object.h>

namespace apc
{
bool BTTestLookForObject::parseGoal(const apc_objects_detection_action::LookForObjectGoalConstPtr &goal)
{
  ROS_INFO("Look for object goal contents:\ncameraID: %s\nmethodID: %s\nobjectslist:", goal->cameraID.c_str(),
           goal->methodID.c_str());
  for (int i = 0; i < goal->objectslist.size(); i++)
  {
    std::cout << goal->objectslist[i] << std::endl;
  }
  return true;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "look_for_object_action_test");
  apc::BTTestLookForObject action(ros::this_node::getName(), "lookforobject");
  ros::spin();
  return 0;
}
