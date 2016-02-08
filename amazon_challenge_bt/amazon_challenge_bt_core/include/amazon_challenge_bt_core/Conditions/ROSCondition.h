#ifndef ROSCONDITION_H
#define ROSCONDITION_H

#include <amazon_challenge_bt_core/ConditionNode.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <amazon_challenge_bt_actions/BTAction.h>
namespace BT
{
    class ROSCondition : public ConditionNode
    {
    public:
        // Constructor
        ROSCondition(std::string Name);
        ~ROSCondition();

        // The method that is going to be executed by the thread
        void Exec();
	  amazon_challenge_bt_actions::BTResult node_result;
          amazon_challenge_bt_actions::BTGoal goal;
    };
}

#endif
