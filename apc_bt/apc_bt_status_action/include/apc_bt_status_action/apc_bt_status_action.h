#ifndef __APC_BT_STATUS_ACTION__
#define __APC_BT_STATUS_ACTION__
#include <ros/ros.h>
#include <behavior_trees/rosaction.h>
#include <apc_bt_comms/TaskManager.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
namespace apc
{
/**
  This class implements a method that calls the task manager service with a
  given status. The status is a constructor argument.
*/
class BTStatusAction : ROSAction
{
public:
  /**
    Class constructor
  */
  BTStatusAction(std::string status, std::string node_name);

  // From ROSAction
  // void initialize();
  // void finalize();
  /**
    Implements the desired Behavior Tree action. When tick'ed, it calls the
    task manager service with its assigned status
  */
  int executeCB(ros::Duration dt);
  // void resetCB();

  /*
      Checks if BT action execution is allowed
  */
  bool isSystemActive();

private:
  std::string status_;
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  apc_bt_comms::TaskManager srv_;
};

/**
  Sets up the boost program options library to parse command line arguments
*/
std::string readCmdLineOption(int argc, char** argv);
}
#endif
