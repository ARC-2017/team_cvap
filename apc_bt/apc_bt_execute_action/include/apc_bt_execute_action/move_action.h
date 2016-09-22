#ifndef __APC_BT_MOVE_ACTION__
#define __APC_BT_MOVE_ACTION__

#include <apc_bt_execute_action/execute_action.h>
#include <apc_manipulation/MoveArmAction.h>

namespace apc
{
/**
    Class that calls the move actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTMoveAction : BTExecuteAction<apc_manipulation::MoveArmAction, apc_manipulation::MoveArmGoal, apc_manipulation::MoveArmResultConstPtr>
{
public:
  BTMoveAction(std::string node_name, std::string actionlib_name)
    : BTExecuteAction<apc_manipulation::MoveArmAction, apc_manipulation::MoveArmGoal, apc_manipulation::MoveArmResultConstPtr>::BTExecuteAction(node_name,
                                                                                                                                                actionlib_name)
  {
  }

  BTMoveAction(std::string node_name, std::string actionlib_name, std::string bt_name)
    : BTExecuteAction<apc_manipulation::MoveArmAction, apc_manipulation::MoveArmGoal, apc_manipulation::MoveArmResultConstPtr>::BTExecuteAction(
          node_name, actionlib_name, bt_name)
  {
  }

protected:
  /*
    Fills in the move action goal.
  */
  bool fillGoal(apc_manipulation::MoveArmGoal &goal);

  /*
    Returns the move timeout value
  */
  double getTimeoutValue();
  /*
    Returns the implemented action's preemption timeout value
  */
  double getPreemptTimeoutValue();
};
}
#endif
