#include <apc_bt_execute_action/simtrack_loader.h>

namespace apc
{
bool SimtrackLoader::fillGoal(apc_objects_detection_action::LookForObjectGoal &goal)
{
  std::vector<std::string> list;
  std::string target;
  std::string mode;

  if (!fillParameter("/apc/task_manager/target_object", target))
  {
    return false;
  }

  if (!first_time_)
  {
    return false;
  }
  else
  {
    goal.targetObj = target;
    goal.globalSearch = false;
    goal.cameraID = "kinect_head";
  }

  first_time_ = false;
  return true;
}

bool SimtrackLoader::processResult(const apc_objects_detection_action::LookForObjectResultConstPtr &result)
{
    return true;
}

double SimtrackLoader::getTimeoutValue()
{
  double timeout;

  fillParameter("/apc/bt/look_action/timeout_kinect", 30.0, timeout);

  return timeout;
}

double SimtrackLoader::getPreemptTimeoutValue()
{
  double timeout;
  fillParameter("/apc/bt/look_action/timeout", 3.0, timeout);

  return timeout;
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "unload_simtrack_models");
  apc::SimtrackLoader action(ros::this_node::getName(), "lookforobject", "unload_simtrack_models_baxter");
  ros::spin();
  return 0;
}
