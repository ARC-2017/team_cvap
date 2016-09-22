#include <apc_bt_status_action/apc_bt_status_action.h>

po::options_description desc("Allowed options");

std::string readCmdLineOption(int argc, char **argv)
{
  desc.add_options()("status", po::value<std::string>(), "Sets status value");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  std::string return_string = "";
  if (vm.count("status"))
  {
    return_string = vm["status"].as<std::string>();
  }
  return return_string;
}

namespace apc
{
bool BTStatusAction::isSystemActive()
{
  bool running;
  if (nh_.hasParam("/apc/task_manager/running"))
  {
    nh_.getParam("/apc/task_manager/running", running);

    if (running)
    {
      return true;
    }
  }
  else
  {
    ROS_ERROR("The parameter /apc/task_manager/running must be set"
              "for a BT action to run!");
  }
  return false;
}

BTStatusAction::BTStatusAction(std::string status, std::string node_name) : ROSAction::ROSAction(node_name)
{
  nh_ = ros::NodeHandle("~");
  client_ = nh_.serviceClient<apc_bt_comms::TaskManager>("/apc/task_manager");
  status_ = status;
}

int BTStatusAction::executeCB(ros::Duration dt)
{
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  srv_.request.status = status_;
  if (client_.call(srv_))
  {
    if (srv_.response.return_val == true)
    {
      // The task manager wants this to happen
      set_feedback(SUCCESS);
      return 1;
    }
    else
    {
      set_feedback(FAILURE);
      return -1;
    }
  }
  else
  {
    set_feedback(FAILURE);
    ROS_ERROR("Failed to call task manager service!");
    return -1;
  }
}
}

int main(int argc, char **argv)
{
  // initialize the behavior tree server node
  std::string status = readCmdLineOption(argc, argv);

  if (status.length() < 1)
  {
    // ROS_ERROR("Must set the desired status through the command line");
    return -1;
  }
  ros::init(argc, argv, "apc_bt_status_action_" + status + "_baxter");  // name used for bt.txt
  apc::BTStatusAction server(status, ros::this_node::getName());
  ROS_INFO("Node name: %s", ros::this_node::getName().c_str());
  ros::spin();
  return 0;
}
