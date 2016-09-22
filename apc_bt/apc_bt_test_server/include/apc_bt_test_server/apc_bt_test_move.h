#ifndef __APC_BT_TEST_MOVE__
#define __APC_BT_TEST_MOVE__

#include <apc_bt_test_server/apc_bt_test_server.h>
#include <apc_manipulation/MoveArmAction.h>

namespace apc
{
/**
    Class that calls the move actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTTestMove : BTTestServer<apc_manipulation::MoveArmAction, apc_manipulation::MoveArmGoalConstPtr,
                                apc_manipulation::MoveArmFeedback, apc_manipulation::MoveArmResult>
{
public:
  BTTestMove(std::string node_name, std::string actionlib_name)
    : BTTestServer<apc_manipulation::MoveArmAction, apc_manipulation::MoveArmGoalConstPtr,
                   apc_manipulation::MoveArmFeedback, apc_manipulation::MoveArmResult>::BTTestServer(node_name,
                                                                                                     actionlib_name)
  {
  }

  ~BTTestMove()
  {
  }

protected:
  bool parseGoal(const apc_manipulation::MoveArmGoalConstPtr &goal);
};
}
#endif
