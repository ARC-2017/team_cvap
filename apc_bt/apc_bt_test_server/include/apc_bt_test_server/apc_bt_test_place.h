#ifndef __APC_BT_TEST_PLACE__
#define __APC_BT_TEST_PLACE__

#include <apc_bt_test_server/apc_bt_test_server.h>
#include <apc_manipulation/PlaceObjectAction.h>

namespace apc
{
/**
    Class that calls the place actionlib server and monitors its feedback status for the high level logic part of
   the system.
*/
class BTTestPlace : BTTestServer<apc_manipulation::PlaceObjectAction, apc_manipulation::PlaceObjectGoalConstPtr,
                                 apc_manipulation::PlaceObjectFeedback, apc_manipulation::PlaceObjectResult>
{
public:
  BTTestPlace(std::string node_name, std::string actionlib_name)
    : BTTestServer<apc_manipulation::PlaceObjectAction, apc_manipulation::PlaceObjectGoalConstPtr,
                   apc_manipulation::PlaceObjectFeedback,
                   apc_manipulation::PlaceObjectResult>::BTTestServer(node_name, actionlib_name)
  {
  }

  ~BTTestPlace()
  {
  }

protected:
  bool parseGoal(const apc_manipulation::PlaceObjectGoalConstPtr &goal);
};
}
#endif
