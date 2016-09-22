#ifndef __APC_BT_LOOK_FOR_OBJECT_ACTION__
#define __APC_BT_LOOK_FOR_OBJECT_ACTION__

#include <apc_bt_execute_action/execute_action.h>
#include <apc_objects_detection_action/LookForObjectAction.h>

namespace apc
{
/**
    Class that calls the perception actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTLookForObjectAction
    : BTExecuteAction<apc_objects_detection_action::LookForObjectAction, apc_objects_detection_action::LookForObjectGoal,
                      apc_objects_detection_action::LookForObjectResultConstPtr>
{
public:
  BTLookForObjectAction(std::string node_name, std::string actionlib_name)
    : BTExecuteAction<apc_objects_detection_action::LookForObjectAction,
                      apc_objects_detection_action::LookForObjectGoal,
                      apc_objects_detection_action::LookForObjectResultConstPtr>::BTExecuteAction(node_name, actionlib_name)
  {
  }

  BTLookForObjectAction(std::string node_name, std::string actionlib_name, std::string bt_name)
    : BTExecuteAction<apc_objects_detection_action::LookForObjectAction,
                      apc_objects_detection_action::LookForObjectGoal,
                      apc_objects_detection_action::LookForObjectResultConstPtr>::BTExecuteAction(node_name, actionlib_name,
                                                                                        bt_name)
  {
  }

protected:
  /*
    Fills in the goal for a particular action.
  */
  bool fillGoal(apc_objects_detection_action::LookForObjectGoal &goal);
  /*
    Process the result.
  */
  bool processResult(const apc_objects_detection_action::LookForObjectResultConstPtr &result);
  /*
    Returns the implemented action's timeout value
  */
  double getTimeoutValue();
  /*
    Returns the implemented action's preemption timeout value
  */
  double getPreemptTimeoutValue();

  bool nameIsInList(std::vector<std::string> list, std::string name)
  {
    for (int i = 0; i < list.size(); i++)
    {
      if (list[i] == name)
      {
        return true;
      }
    }

    return false;
  }
};
}
#endif
