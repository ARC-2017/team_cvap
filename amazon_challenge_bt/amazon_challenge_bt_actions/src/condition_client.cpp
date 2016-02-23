#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amazon_challenge_bt_actions/BTAction.h>


int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_condition");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<amazon_challenge_bt_actions::BTAction> ac("listdone", true);
	amazon_challenge_bt_actions::BTResult node_result;
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  amazon_challenge_bt_actions::BTGoal goal;
  //goal.order = 20;


int command=0;
bool isRunning = false;



 	while(command!=3){

	ROS_INFO("Send a command: 1:start the action | 2:stop the action | 3:exit the program");
	std::cin >> command;

		switch(command){
			case 1: 
                if(!isRunning){
                    ROS_INFO("I am running the request");
	  				ac.sendGoal(goal);
					//ac.ClientGoalHandle();
                    isRunning=true;
                    ac.waitForResult(ros::Duration(30.0));
                    node_result = *(ac.getResult());
                     ROS_INFO("Action finished, status: %d",node_result.status);
                }else{
                    ROS_INFO("I am re-running the request");
                    ac.cancelGoal();
                    ac.sendGoal(goal);
                    ac.waitForResult(ros::Duration(30.0));
                    node_result = *(ac.getResult());

                     ROS_INFO("Action finished, status: %d",node_result.status);
                }
			break;
			case 2: 
                ROS_INFO("I am cancelling the request");
				ac.cancelGoal();
				isRunning=false;
			break;
			default:
			break;
		}




	}

return 0;

}
/*

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  amazon_challenge_bt_actions::BTGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  //bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
 ros::Duration(0.5).sleep(); 
    ROS_INFO("I am cancelling the request");
ac.cancelGoal();
 ros::Duration(0.5).sleep(); 
  ac.sendGoal(goal);

bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;

*/
