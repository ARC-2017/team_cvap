#ifndef __APC_BT_PLACE_ACTION__
#define __APC_BT_PLACE_ACTION__

#include <apc_bt_execute_action/execute_action.h>
#include <apc_manipulation/PlaceObjectAction.h>

namespace apc
{
/**
    Class that calls the place actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTPlaceAction : BTExecuteAction<apc_manipulation::PlaceObjectAction, apc_manipulation::PlaceObjectGoal, apc_manipulation::PlaceObjectResultConstPtr>
{
public:
  BTPlaceAction(std::string node_name, std::string actionlib_name, std::string bt_name)
    : BTExecuteAction<apc_manipulation::PlaceObjectAction, apc_manipulation::PlaceObjectGoal, apc_manipulation::PlaceObjectResultConstPtr>::BTExecuteAction(
          node_name, actionlib_name, bt_name)
  {
  }

protected:
  /*
    Fills in the place action goal.
  */
  bool fillGoal(apc_manipulation::PlaceObjectGoal &goal);

  /*
    Returns the place action timeout value
  */
  double getTimeoutValue();
  /*
    Returns the implemented action's preemption timeout value
  */
  double getPreemptTimeoutValue();
};
}
#endif
