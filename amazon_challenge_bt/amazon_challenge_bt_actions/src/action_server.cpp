#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amazon_challenge_bt_actions/BTAction.h>


enum Status { RUNNING, SUCCESS, FAILURE};


class BTAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<amazon_challenge_bt_actions::BTAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  amazon_challenge_bt_actions::BTFeedback feedback_;
  amazon_challenge_bt_actions::BTResult result_;

public:

  BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~BTAction(void)
  {
  }
  //void executeCB()
  void executeCB(const amazon_challenge_bt_actions::BTGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    // publish info to the console for the user
    ROS_INFO("Starting Action");

    // start executing the action
    for(int i=1; i<=5; i++)
    {

     // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Action Halted");

        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      ROS_INFO("Executing Action");


      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
        setStatus(SUCCESS);
    }
  }




//returns the status to the client (Behavior Tree)
  void setStatus(int status){
      //Set The feedback and result of BT.action
      feedback_.status = status;
      result_.status = feedback_.status;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // setSucceeded means that it has finished the action (it has returned SUCCESS or FAILURE).
      as_.setSucceeded(result_);

      switch(status){//Print for convenience
      case SUCCESS:
        ROS_INFO("Action %s Succeeded", ros::this_node::getName().c_str() );
        break;
      case FAILURE:
          ROS_INFO("Action %s Failed", ros::this_node::getName().c_str() );
        break;
          ROS_ERROR("Action %s has a wrong return status", ros::this_node::getName().c_str() );
      default:
        break;
      }
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "action");
      ROS_INFO(" Enum: %d",RUNNING);
      ROS_INFO(" Action Ready for Ticks");
  BTAction bt_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
