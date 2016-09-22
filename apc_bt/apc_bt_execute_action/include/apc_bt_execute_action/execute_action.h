#ifndef __APC_BT_EXECUTE_ACTION__
#define __APC_BT_EXECUTE_ACTION__
#include <ros/ros.h>
#include <apc_bt_comms/TaskManager.h>
#include <behavior_trees/rosaction.h>
#include <actionlib/client/simple_action_client.h>

namespace apc
{
const std::string left_arm = "left_arm";
const std::string right_arm = "right_arm";
const std::string both_arms = "both_arms";

const std::string bin_A = "bin_A";
const std::string bin_B = "bin_B";
const std::string bin_C = "bin_C";
const std::string bin_D = "bin_D";
const std::string bin_E = "bin_E";
const std::string bin_F = "bin_F";
const std::string bin_G = "bin_G";
const std::string bin_H = "bin_H";
const std::string bin_I = "bin_I";
const std::string bin_J = "bin_J";
const std::string bin_K = "bin_K";
const std::string bin_L = "bin_L";
const std::string home = "HOME";
const std::string tote = "tote";

enum Desperation {level_1 = 1, level_2 = 2, level_3 = 3, level_4 = 4};

/*
    Fills in a generic parameter from the parameter server
*/
template<typename T>
bool fillParameter(std::string param_name, T &param_val);
bool fillParameter(std::string param_name, apc::Desperation &param_val);

template<typename T>
void fillParameter(std::string param_name, T def, T &param_val);

template<typename T>
void setParameter(std::string param_name, T param_val);

/**
  This class implements an interface for BT actions destined to call rosaction
  servers implemented by the other apc projects. The class template is the
  actionlib action class that will be used by the implementation
*/
template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
class BTExecuteAction : ROSAction
{
public:
  BTExecuteAction(std::string node_name, std::string actionlib_name);

  /*
    Alternative constructor for BT actions that share the same actionlib server
  */
  BTExecuteAction(std::string node_name, std::string actionlib_name, std::string bt_name);
  virtual ~BTExecuteAction();

  int executeCB(ros::Duration dt);

protected:
  /*
    Fills in the goal for a particular action.
  */
  virtual bool fillGoal(ActionGoal &goal) = 0;
  /*
    Process the result message
  */
  virtual bool processResult(const ActionResultConstPtr &result);
  /*
    Returns the implemented action's timeout value
  */
  virtual double getTimeoutValue() = 0;
  /*
    Returns the implemented action's preemption timeout value
  */
  virtual double getPreemptTimeoutValue() = 0;
  /*
      Checks if BT action execution is allowed
  */
  bool isSystemActive();

private:
  actionlib::SimpleActionClient<ActionClass> *action_client_;
  ActionGoal goal_;
  std::string action_name_;
  std::string bt_name_;
};

// Template classes "must" be implemented in the header files... -->
// http://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file

// Notice that the execute action is appending "_baxter" to the given node name.
// This allows for us to maintain the comm actionlib server and the inner BT
// actionlib server. Yes Joshua, I know it's bad design :)
template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
BTExecuteAction<ActionClass, ActionGoal, ActionResultConstPtr>::BTExecuteAction(std::string node_name, std::string actionlib_name)
  : ROSAction::ROSAction(actionlib_name + std::string("_baxter"))
{
  nh_ = ros::NodeHandle("~");
  action_name_ = actionlib_name;
}

template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
BTExecuteAction<ActionClass, ActionGoal, ActionResultConstPtr>::BTExecuteAction(std::string node_name, std::string actionlib_name,
                                                          std::string bt_name)
  : ROSAction::ROSAction(bt_name)
{
  nh_ = ros::NodeHandle("~");
  action_name_ = actionlib_name;
  bt_name_ = bt_name;
}

template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
BTExecuteAction<ActionClass, ActionGoal, ActionResultConstPtr>::~BTExecuteAction()
{
  delete action_client_;
}

template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
bool BTExecuteAction<ActionClass, ActionGoal, ActionResultConstPtr>::isSystemActive()
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

template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
bool BTExecuteAction<ActionClass, ActionGoal, ActionResultConstPtr>::processResult(const ActionResultConstPtr &result)
{
    // Dummy implementation
    return true;
}

template <class ActionClass, class ActionGoal, class ActionResultConstPtr>
int BTExecuteAction<ActionClass, ActionGoal, ActionResultConstPtr>::executeCB(ros::Duration dt)
{
  action_client_ = new actionlib::SimpleActionClient<ActionClass>(action_name_, true);
  set_feedback(RUNNING);

  if (!isSystemActive())
  {
    return 1;
  }

  bool active_server = action_client_->waitForServer(ros::Duration(2.0));

  if (!active_server)
  {
    ROS_ERROR("Actionlib server failed to start for action %s!", action_name_.c_str());
    set_feedback(FAILURE);
    return 1;
  }

  bool has_parameters = fillGoal(goal_);

  if (!has_parameters)
  {
    ROS_ERROR("Failed to get parameters for action %s!", action_name_.c_str());
    set_feedback(FAILURE);
    return 1;
  }

  ROS_INFO("Sending goal from action: %s. Timeout value: %.2f", action_name_.c_str(), getTimeoutValue());
  nh_.setParam("/apc/bt/action", bt_name_);

  action_client_->sendGoal(goal_);
  bool finished = action_client_->waitForResult(ros::Duration(getTimeoutValue()));

  if (finished)
  {
    // Add success logic here
    actionlib::SimpleClientGoalState state = action_client_->getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());

    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      processResult(action_client_->getResult());
      set_feedback(SUCCESS);
    }
    else
    {
      set_feedback(FAILURE);
    }
  }
  else
  {
    ROS_ERROR("Action %s timeout! Cancelling goals...", action_name_.c_str());
    action_client_->cancelAllGoals();
    finished = action_client_->waitForResult(ros::Duration(getPreemptTimeoutValue()));

    if (!finished)
    {
        ROS_ERROR("Action %s did not preempt in less than %.2f seconds!", action_name_.c_str(), getPreemptTimeoutValue());
    }

    set_feedback(FAILURE);
  }

  return 1;
}


template<typename T>
bool fillParameter(std::string param_name, T &param_val)
{
  if (ros::param::has(param_name.c_str()))
  {
    ros::param::get(param_name.c_str(), param_val);
  }
  else
  {
    ROS_ERROR("%s not set!", param_name.c_str());
    return false;
  }
  return true;
}

bool fillParameter(std::string param_name, apc::Desperation &param_val)
{
  int desp_int;
  if (ros::param::has(param_name.c_str()))
  {
    ros::param::get(param_name.c_str(), desp_int);

    switch (desp_int)
    {
      case 1:
        param_val = level_1;
        break;
      case 2:
        param_val = level_2;
        break;
      case 3:
        param_val = level_3;
        break;
      default:
        param_val = level_4;
    }
  }
  else
  {
    ROS_ERROR("%s not set!", param_name.c_str());
    return false;
  }
  return true;
}

template<typename T>
void fillParameter(std::string param_name, T def, T &param_val)
{
  if (ros::param::has(param_name.c_str()))
  {
    ros::param::get(param_name.c_str(), param_val);
  }
  else
  {
    param_val = def;
    ROS_WARN("Param '%s' not set. Using default.", param_name.c_str());
  }
}

template<typename T>
void setParameter(std::string param_name, T param_val)
{
  ros::param::set(param_name, param_val);
}
}
#endif
