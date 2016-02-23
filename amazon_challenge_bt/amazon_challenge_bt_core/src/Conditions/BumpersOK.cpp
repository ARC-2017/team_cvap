#include <amazon_challenge_bt_core/Conditions/BumpersOK.h>

using namespace BT;


enum Status {RUNNING,SUCCESS, FAILURE};


BumpersOK::BumpersOK(std::string Name) : ConditionNode::ConditionNode(Name)
{
    Type = Condition;

    // Thread start
    Thread = boost::thread(&BumpersOK::Exec, this);
}

BumpersOK::~BumpersOK() {}

void BumpersOK::Exec()
{

    ROS_INFO("Waiting For the Acutator bumpersok to start");

    actionlib::SimpleActionClient<amazon_challenge_bt_actions::BTAction> ac("bumpersok", true);
    ac.waitForServer(); //will wait for infinite time until the server starts
    ROS_INFO("Actuator bumpersok  Started");

    amazon_challenge_bt_actions::BTGoal goal;
    while(true)
    {
	
        // Waiting for a tick to come
        Semaphore.Wait();
        node_result.status = 0;

        ROS_INFO("I am running the request");


        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Condition checking and state update
       // do{
            ac.sendGoal(goal);
            ac.waitForResult(ros::Duration(30.0));
            node_result = *(ac.getResult());

            std::cout << " Condition Status" << node_result.status << "!" << std::endl;


       // }while(node_result.status != 2 && node_result.status != 1 ); //if it is not halted and has not returned a status

        if (node_result.status == SUCCESS)
        {
            SetNodeState(Success);
           //std::cout << Name << " returning Success" << Success << "!" << std::endl;
        }
        else if( node_result.status == FAILURE)
        {
            SetNodeState(Failure);
           // std::cout << Name << " returning Failure" << Failure << "!" << std::endl;
        }else{
            SetNodeState(Failure);
            std::cout << Name << " returning NOTHING" << Failure << "!" << std::endl;
        }

	

        // Resetting the state
        WriteState(Idle);
    }
}
