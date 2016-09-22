#ifndef __APC_BT_TEST_PICK__
#define __APC_BT_TEST_PICK__

#include <apc_bt_test_server/apc_bt_test_server.h>
#include <apc_manipulation/PickObjectAction.h>

namespace apc
{
/**
    Class that calls the pick actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTTestPick : BTTestServer<apc_manipulation::PickObjectAction, apc_manipulation::PickObjectGoalConstPtr,
                                apc_manipulation::PickObjectFeedback, apc_manipulation::PickObjectResult>
{
public:
  BTTestPick(std::string node_name, std::string actionlib_name)
    : BTTestServer<apc_manipulation::PickObjectAction, apc_manipulation::PickObjectGoalConstPtr,
                   apc_manipulation::PickObjectFeedback,
                   apc_manipulation::PickObjectResult>::BTTestServer(node_name, actionlib_name)
  {
  }

  ~BTTestPick()
  {
  }

protected:
  bool parseGoal(const apc_manipulation::PickObjectGoalConstPtr &goal);
};
}
#endif
