#ifndef ACTIONTEST_H
#define ACTIONTEST_H

#include <amazon_challenge_bt_core/ActionNode.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amazon_challenge_bt_actions/BTAction.h>
namespace BT
{
    class ActionTestNode : public ActionNode
    {
    public:
NodeState status_;
        // Constructor
        ActionTestNode(std::string Name);
        ~ActionTestNode();

        // The method that is going to be executed by the thread
        void Exec();
        void SetBehavior(NodeState status);

        // The method used to interrupt the execution of the node
        bool Halt();

  	//actionlib::SimpleActionClient<amazon_challenge_bt_actions::BTAction> ac();
	
	  amazon_challenge_bt_actions::BTResult node_result;
          amazon_challenge_bt_actions::BTGoal goal;
    };
}

#endif
