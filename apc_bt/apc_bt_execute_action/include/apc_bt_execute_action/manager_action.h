#ifndef __APC_BT_MANAGER_ACTION__
#define __APC_BT_MANAGER_ACTION__

#include <apc_bt_execute_action/execute_action.h>
#include <apc_bt_comms/ManagerAction.h>

namespace apc
{
/**
    Class that calls the perception actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTManagerAction : BTExecuteAction<apc_bt_comms::ManagerAction, apc_bt_comms::ManagerGoal, apc_bt_comms::ManagerResultConstPtr>
{
public:
  BTManagerAction(std::string node_name, std::string actionlib_name)
    : BTExecuteAction<apc_bt_comms::ManagerAction, apc_bt_comms::ManagerGoal,
                      apc_bt_comms::ManagerResultConstPtr>::BTExecuteAction(node_name, actionlib_name)
  {
  }

  BTManagerAction(std::string node_name, std::string actionlib_name, std::string bt_name)
    : BTExecuteAction<apc_bt_comms::ManagerAction, apc_bt_comms::ManagerGoal, apc_bt_comms::ManagerResultConstPtr>::BTExecuteAction(node_name,
                                                                                                                    actionlib_name, bt_name)
  {
  }

protected:
  /*
    Fills in the goal for a particular action.
  */
  bool fillGoal(apc_bt_comms::ManagerGoal &goal);

  /*
    Returns the implemented action's timeout value
  */
  double getTimeoutValue();
  
  /*
    Returns the implemented action's preemption timeout value
  */
  double getPreemptTimeoutValue();
};
}
#endif
