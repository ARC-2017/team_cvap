#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>//actionlib
#include <amazon_challenge_bt_actions/BTAction.h>//Definition of action. see /scr/folder





enum Status { RUNNING, SUCCESS, FAILURE};


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
  AL::ALMotionProxy* motion_proxy_ptr;

  BTAction(std::string name) :
    as_(nh_, name, boost::bind(&BTAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    robot_ip = "192.168.0.188";
    std::cout << "Robot ip to use is: " << robot_ip << std::endl;
    motion_proxy_ptr = new AL::ALMotionProxy("192.168.0.188", 9559);


    //put this in init()

    AL::ALValue stiffness_name("Body");
    AL::ALValue stiffness(1.0f);
    AL::ALValue stiffness_time(1.0f);
    //motion_proxy_ptr->stiffnessInterpolation(stiffness_name,
                                           //  stiffness,
                                            // stiffness_time);
    //motion_proxy_ptr->moveInit();

  }

  ~BTAction(void)
  {
      //delete motion_proxy_ptr;//closes the NAO proxy

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
    for(int i=1; i<=20; i++)
    {

    ROS_INFO("Executing Action");

    //motion_proxy_ptr->post.move(0.1, 0.0, 0.0);

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Action Halted");


      // motion_proxy_ptr->stopMove();


        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.status = RUNNING;
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes

      r.sleep();
   }

    if(success)
    {
      result_.status = feedback_.status;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveforward");
      ROS_INFO(" Enum: %d",RUNNING);
      ROS_INFO(" Action  moveforward Ready for Ticks");
  BTAction bt_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
