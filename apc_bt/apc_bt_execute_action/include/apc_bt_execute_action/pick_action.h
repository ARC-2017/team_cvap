#ifndef __APC_BT_PICK_ACTION__
#define __APC_BT_PICK_ACTION__

#include <apc_bt_execute_action/execute_action.h>
#include <apc_manipulation/PickObjectAction.h>

namespace apc
{
/**
    Class that calls the pick actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTPickAction : BTExecuteAction<apc_manipulation::PickObjectAction, apc_manipulation::PickObjectGoal, apc_manipulation::PickObjectResultConstPtr>
{
public:
  BTPickAction(std::string node_name, std::string actionlib_name, std::string bt_name)
    : BTExecuteAction<apc_manipulation::PickObjectAction, apc_manipulation::PickObjectGoal, apc_manipulation::PickObjectResultConstPtr>::BTExecuteAction(
          node_name, actionlib_name, bt_name)
  {
  }

protected:
  /*
    Fills in the pick goal.
  */
  bool fillGoal(apc_manipulation::PickObjectGoal &goal);

  /*
    Returns the pick action timeout value
  */
  double getTimeoutValue();
  /*
    Returns the implemented action's preemption timeout value
  */
  double getPreemptTimeoutValue();
};
}
#endif
