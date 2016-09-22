#include <apc_bt_execute_action/set_kinect_action.h>

namespace apc
{
SetKinectAction::SetKinectAction(std::string node_name, std::string bt_name) : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
}

int SetKinectAction::executeCB(ros::Duration dt)
{
  std::string object_name;
  set_feedback(RUNNING);

  fillParameter("/apc/task_manager/target_object", object_name);
  setParameter("/apc/task_manager/memory/" + object_name, true);

  set_feedback(SUCCESS);
  return 1;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_memory_kinect");
  apc::SetKinectAction action(ros::this_node::getName(), "set_memory_kinect_baxter");
  ros::spin();
  return 0;
}
