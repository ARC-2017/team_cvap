/*
 *
 * Sergio Caccamo (caccamo@kth.se) - KTH Royal Institute of Technology
 *
 * Development framework for APC 2016 - Action server lookforobject
 * V 1.0 (under development) April-2016
 *
 * Description: Client example for the apc_objects_detection_server
 *
 * 
 *  Copyright (c) 2016, Sergio Caccamo, Francisco Vina, CVAP, KTH
 *    All rights reserved.
 * 
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of KTH nor the
 *         names of its contributors may be used to endorse or promote products
 *          derived from this software without specific prior written permission.
 * 
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 *TODO :        To do
 *TOCHK:        To check
 *TOCNG:        To change
 *TBR:          To be removed
 *
 */



#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <apc_objects_detection_action/LookForObjectAction.h>
#include <boost/thread.hpp>

void spinThread()
{
  ros::spin();
}

int main (int argc, char **argv)
{

  ros::init(argc, argv, "test_lookforobject");
  ros::NodeHandle nh_;
  
  // create the action client
  actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> ac("lookforobject");
  boost::thread spin_thread(&spinThread);

  ROS_INFO("Waiting for LookForObject action server to start.");
  ac.waitForServer();

  ROS_INFO("LookForObject Action server started, sending goal.");
  
  // updating the action server with a new object @TOCHK: change the parameter server
  std::string targetObj, objectslist_, cameraID, methodID, taskID, binID, param_;

  cameraID = "ids_left"; //e.g.
  methodID = "simtrack";
  taskID = "list";
  binID = "bin_H";
  if (argc >= 2) {
    targetObj = argv[1];
  	if (argc >= 3) {
  		binID = argv[2];
  		if (binID == "bin_A" || binID == "bin_B" || binID == "bin_C" || binID == "bin_D" || binID == "bin_E" || binID == "bin_F")
			cameraID = "ids_left";
  	}
  	if (argc == 4) {
  		cameraID = argv[3];
  	}
    }
  else {
    targetObj = "crayola_24_ct";
  }

  
  //nh_.setParam("/apc/perception/lookforobject/objectlist", objectslist_);
  //nh_.setParam("/apc/perception/lookforobject/cameraID", "kinect_chest");
  //nh_.setParam("/apc/perception/lookforobject/methodID", "simtrack");
  //nh_.setParam("/apc/perception/lookforobject/taskID", "pick_from_shelf");
  
  // send a goal to the action
  apc_objects_detection_action::LookForObjectGoal goal;
  
  goal.goalID = 100; //e.g.
  std::vector<std::string> objectslist(1);
  objectslist[0]=objectslist_;
  goal.objectslist = objectslist;
  goal.targetObj = targetObj;
  goal.cameraID = cameraID;
  goal.methodID = methodID;
  goal.taskID = taskID;
  goal.binID = binID;
  
  ac.sendGoal(goal);

  // wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  // shutdown the node and join the thread back before exiting
  ros::shutdown();
  spin_thread.join();

  //exit
  return 0;
}
