#ifndef __APC_BT_SET_IDS_ACTION__
#define __APC_BT_SET_IDS_ACTION__
#include <ros/ros.h>
#include <apc_bt_comms/TaskManager.h>
#include <apc_bt_execute_action/execute_action.h>
#include <behavior_trees/rosaction.h>

namespace apc
{
class SetIDSAction : ROSAction
{
public:
  SetIDSAction(std::string node_name, std::string bt_name);
  ~SetIDSAction()
  {
  }

  int executeCB(ros::Duration dt);

protected:
  /*
      Checks if BT action execution is allowed
  */
  bool isSystemActive();
};

bool SetIDSAction::isSystemActive()
{
  bool running;
  if (nh_.hasParam("/apc/task_manager/running"))
  {
    nh_.getParam("/apc/task_manager/running", running);

    if (running)
    {
      return true;
    }
  }
  else
  {
    ROS_ERROR("The parameter /apc/task_manager/running must be set"
              "for a BT action to run!");
  }
  return false;
}
}
#endif
