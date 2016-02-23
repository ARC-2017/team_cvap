#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amazon_challenge_bt_actions/BTAction.h>
#include "std_msgs/String.h"


enum Status { RUNNING, SUCCESS, FAILURE};

std::string value;// I should do a mutex here but it is only for demostration now

boost::mutex mtx_; // explicit mutex declaration The value of the condition is read by the main thread but written by the listener


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
    value="FAILURE";
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
    ROS_INFO("Starting Condition");

    // start executing the action
    //for(int i=1; i<=5; i++)
    //{
      ROS_INFO("Checking Condition");
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      //r.sleep();
    //}

      if(value.compare("SUCCESS") == 0)
    {
        setStatus(SUCCESS);
      }else{
        setStatus(FAILURE);
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
        ROS_INFO("Arm posed");
        break;
      case FAILURE:
          ROS_INFO("Arm NOT Posed");
        break;
          ROS_ERROR("Condition %s has a wrong return status", ros::this_node::getName().c_str() );
      default:
        break;
      }
  }


  std::string GetConditionValue() {
      boost::lock_guard<boost::mutex> guard(mtx_);
      return value;
  }

};

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
 boost::lock_guard<boost::mutex> guard(mtx_);
 //ROS_INFO("I heard: [%s]", msg->data.c_str());
 value = msg->data.c_str();
}

int main(int argc, char** argv)
{

    std::string condition_name = "arm_posed";
    ros::init(argc, argv, condition_name);
    ROS_INFO(" Enum: %d",RUNNING);
    ROS_INFO(" Condition Ready for Ticks");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(condition_name, 1000, chatterCallback);

    BTAction bt_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
