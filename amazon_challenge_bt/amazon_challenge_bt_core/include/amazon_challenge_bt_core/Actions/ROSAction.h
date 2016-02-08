#ifndef ROSACTION_H
#define ROSACTION_H

#include <amazon_challenge_bt_core/ActionNode.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amazon_challenge_bt_actions/BTAction.h>
namespace BT
{
    class ROSAction : public ActionNode
    {
    public:
        // Constructor
        ROSAction(std::string Name);
        ~ROSAction();

        // The method that is going to be executed by the thread
        void Exec();

        // The method used to interrupt the execution of the node
        bool Halt();


        std::string getProcessName();

        void startProcess();
        void stopProcess();
        void stopAllOtherProcesses();
        void startAllOtherProcesses();

  	//actionlib::SimpleActionClient<bt_actions::BTAction> ac();

	  amazon_challenge_bt_actions::BTResult node_result;
          amazon_challenge_bt_actions::BTGoal goal;
    };
}

#endif
