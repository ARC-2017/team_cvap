#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>//actionlib
#include <amazon_challenge_bt_actions/BTAction.h>//Definition of action. see /scr/folder


#include <iostream>
#include <unistd.h>
#include <math.h>

//NAO Libraries
#include <alproxies/almemoryproxy.h>
#include <alproxies/alsensorsproxy.h>
#include <alerror/alerror.h>



enum Status {RUNNING,SUCCESS, FAILURE};//BT return status


class BTAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<amazon_challenge_amazon_challenge_bt_actions::BTAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  amazon_challenge_bt_actions::BTFeedback feedback_;
  amazon_challenge_bt_actions::BTResult result_;
  std::string robot_ip;


public:
    float l_bumper; //reading of the left bumper
    float r_bumper; //reading of the left bumper

    //Declaring NAO AL proxies
     AL::ALMemoryProxy* memory_proxy_ptr;
     AL::ALSensorsProxy* sensors_proxy_ptr;
    BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::executeCB, this, _1), false),
    action_name_(name)
    {
        as_.start();
        robot_ip = "192.168.0.188";
        std::cout << "Robot ip to use is: " << robot_ip << std::endl;
        sensors_proxy_ptr = new AL::ALSensorsProxy(robot_ip, 9559);
        memory_proxy_ptr = new AL::ALMemoryProxy(robot_ip, 9559);
        ROS_INFO("Condition Server Started");
    }

    ~BTAction(void)
    {

    }
    void executeCB(const amazon_challenge_bt_actions::BTGoalConstPtr &goal)
    {


    // publish info to the console for the user
    ROS_INFO("Checking Bumpers");
    {

        l_bumper = memory_proxy_ptr->getData("LeftBumperPressed");
        r_bumper = memory_proxy_ptr->getData("RightBumperPressed");
        ROS_INFO("Executing Condition");
        if(l_bumper > 0.5f || r_bumper > 0.5f){
        ROS_INFO("Bumper(s) Hit");
        setStatus(FAILURE);

        }else{
        ROS_INFO("Bumper(s) OK");
        setStatus(SUCCESS);
         }
    }

    }



    void setStatus(int status){
        feedback_.status = status;
        result_.status = feedback_.status;

          // publish the feedback
        as_.publishFeedback(feedback_);
        as_.setSucceeded(result_);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bumpersok");
    ROS_INFO(" Enum: %d",RUNNING);
    ROS_INFO(" BumpersOK Ready for Ticks");
    BTAction bt_action(ros::this_node::getName());
    ros::spin();
    return 0;
}
