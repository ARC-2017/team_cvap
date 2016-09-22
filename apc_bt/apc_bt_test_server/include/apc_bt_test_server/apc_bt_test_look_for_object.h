#ifndef __APC_BT_TEST_LOOK_FOR_OBJECT__
#define __APC_BT_TEST_LOOK_FOR_OBJECT__

#include <apc_bt_test_server/apc_bt_test_server.h>
#include <apc_objects_detection_action/LookForObjectAction.h>

namespace apc
{
/**
    Class that calls the look_for_object actionlib server and monitors its feedback status for the high level logic part
   of
   the system.
*/
class BTTestLookForObject
    : BTTestServer<apc_objects_detection_action::LookForObjectAction, apc_objects_detection_action::LookForObjectGoalConstPtr,
                   apc_objects_detection_action::LookForObjectFeedback, apc_objects_detection_action::LookForObjectResult>
{
public:
  BTTestLookForObject(std::string node_name, std::string actionlib_name)
    : BTTestServer<apc_objects_detection_action::LookForObjectAction, apc_objects_detection_action::LookForObjectGoalConstPtr,
                   apc_objects_detection_action::LookForObjectFeedback,
                   apc_objects_detection_action::LookForObjectResult>::BTTestServer(node_name, actionlib_name)
  {
  }

  ~BTTestLookForObject()
  {
  }

protected:
  bool parseGoal(const apc_objects_detection_action::LookForObjectGoalConstPtr &goal);
};
}
#endif
