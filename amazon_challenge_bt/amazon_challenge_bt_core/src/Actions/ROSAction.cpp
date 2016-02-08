#include <amazon_challenge_bt_core/Actions/ROSAction.h>

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

using namespace BT;

enum Status {RUNNING,SUCCESS, FAILURE};



ROSAction::ROSAction(std::string Name) : ActionNode::ActionNode(Name)
{
    Type = Action;
    // Thread start
    Thread = boost::thread(&ROSAction::Exec, this);




}

ROSAction::~ROSAction() {}

void ROSAction::Exec()
{


    //bool hasReturned = false;
	  // create the action client
	  // true causes the client to spin its own thread
    ROS_INFO("Waiting For the Acutator %s to start", Name.c_str());

      actionlib::SimpleActionClient<amazon_challenge_bt_actions::BTAction> ac(Name, true);

      ac.waitForServer(); //will wait for infinite time until the server starts
      ROS_INFO("Tthe Acutator %s has started", Name.c_str());

      amazon_challenge_bt_actions::BTGoal goal;
      node_result.status = RUNNING;//
    while(true)
    {
        // Waiting for a tick to come
        Semaphore.Wait();

        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Running state
        SetNodeState(Running);
        std::cout << Name << " returning " << Running << "!" << std::endl;
        node_result.status = RUNNING;
        // Perform action...
//            stopAllOtherProcesses();
//            startProcess();
                    ROS_INFO("I am running the request to %s",Name.c_str());

//                    ros::Duration(3.0).sleep();
                    ac.sendGoal(goal);
                    //ac.waitForResult(ros::Duration(30.0));
            do{
                node_result = *(ac.getResult());//checking the result
                ros::Duration(0.1).sleep();
                    } while(node_result.status == RUNNING && ReadState() == Running);
            ROS_INFO("The Server Has Replied");

                    std::cout << Name << " RETURNING " << node_result.status << "!" << std::endl;
//                    startAllOtherProcesses();

        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }


        if (node_result.status == SUCCESS)
        {
            // trying to set the outcome state:
            if (WriteState(Success) != true)
            {
                // meanwhile, my father halted me!
                std::cout << Name << " Halted!" << std::endl;
                ROS_INFO("I am cancelling the request");
                ac.cancelGoal();
                // Resetting the state
                WriteState(Idle);
                continue;
            }

            std::cout << Name << " returning Success " << Success << "!" << std::endl;
        }
        else if( node_result.status == FAILURE)
        {
            // trying to set the outcome state:
            if (WriteState(Failure) != true)
            {
                // meanwhile, my father halted me!
                std::cout << Name << " Halted!" << std::endl;
                ROS_INFO("I am cancelling the request");
                ac.cancelGoal();
                // Resetting the state
                WriteState(Idle);
                continue;
            }

            std::cout << Name << " returning Failure" << Failure << "!" << std::endl;
        }else{

            std::cout << Name << " Halted!" << std::endl;
            ROS_INFO("I am cancelling the request");
            ac.cancelGoal();
            // Resetting the state
            WriteState(Idle);
            continue;

            std::cout << Name << " returning NOTHING (HALTED)" << Failure << "!" << std::endl;
        }


            std::cout << Name << " returning " << Success << "!" << std::endl;


        // Synchronization
        // (my father is telling me that it has read my new state)
        Semaphore.Wait();

        if(ReadState() == Exit)
        {
            // The behavior tree is going to be destroied
            return;
        }

        // Resetting the state
        WriteState(Idle);
    }
}

bool ROSAction::Halt()
{

	  //actionlib::SimpleActionClient<amazon_challenge_bt_actions::BTAction> ac("action", true);
        ROS_INFO("I am Halting the client");
	//ac.cancelGoal();
    // Lock acquistion
    boost::lock_guard<boost::mutex> LockGuard(StateMutex);

    // Checking for "Running" correctness
    if (State != Running)
    {
        return false;
    }

    State = Halted;
    return true;
}


std::string ROSAction::getProcessName()
{
    std::string as_name = Name;
    std::string process_name = Name;

    if (as_name=="base_torso_head_init_server")
    {
        process_name = "base_torso_head";
    }

    else if (as_name=="amazon_detector")
    {
        process_name = "detector";
    }

    else if (as_name=="pregrasp_server")
    {
        process_name = "pregrasp";
    }

    else if (as_name=="grasp_object")
    {
        process_name = "grasping";
    }

    else if (as_name=="drop_object")
    {
        process_name = "dropping";
    }

    else
    {
        process_name = "";
    }

    return process_name;
}

void ROSAction::startProcess()
{
    std::string process_name = getProcessName();

    if(process_name!="")
    {
        std::cout << "starting process " << process_name << std::endl;
        system((std::string("pkill -CONT ") + process_name).c_str());

        if (process_name=="grasping")
        {
            std::cout << "starting process detector" << std::endl;
            system("pkill -CONT detector");

            std::cout << "starting process pr2_cam_switch" << std::endl;
            system("pkill -CONT pr2_cam_switch");

            std::cout << "starting process periodic_cloud" << std::endl;
            system("pkill -CONT periodic_cloud");
        }

        if(process_name=="detector")
        {
            std::cout << "starting process pr2_cam_switch" << std::endl;
            system("pkill -CONT pr2_cam_switch");

            std::cout << "starting process periodic_cloud" << std::endl;
            system("pkill -CONT periodic_cloud");
        }
    }
}

void ROSAction::stopProcess()
{

    std::string process_name = getProcessName();

    if(process_name!="")
    {
        std::cout << "stopping process " << process_name << std::endl;
        system((std::string("pkill -STOP ") + process_name).c_str());

    }
}

void ROSAction::stopAllOtherProcesses()
{
    if(Name!="base_torso_head_init_server")
    {
        std::string process_name = "base_torso_head";
        std::cout << "stopping process " << process_name << std::endl;
        system((std::string("pkill -STOP ") + process_name).c_str());
    }

    if(Name!="amazon_detector")
    {
        std::string process_name = "detector";
        std::cout << "stopping process " << process_name << std::endl;
        system((std::string("pkill -STOP ") + process_name).c_str());
    }

    if(Name!="pregrasp_server")
    {
        std::string process_name = "pregrasp";
        std::cout << "stopping process " << process_name << std::endl;
        system((std::string("pkill -STOP ") + process_name).c_str());
    }

    if(Name!="grasp_object")
    {
        std::string process_name = "grasping";
        std::cout << "stopping process " << process_name << std::endl;
        system((std::string("pkill -STOP ") + process_name).c_str());
    }

    if(Name!="drop_object")
    {
        std::string process_name = "dropping";
        std::cout << "stopping process " << process_name << std::endl;
        system((std::string("pkill -STOP ") + process_name).c_str());
    }


//        std::cout << "stopping process rviz" << std::endl;
//        system("pkill -STOP rviz");


//    std::cout << "stopping process rqt_amazon" << std::endl;
//    system("pkill -STOP rqt_amazon");

    if(Name!="amazon_detector")
    {
        std::cout << "stopping process pr2_cam_switch" << std::endl;
        system("pkill -STOP pr2_cam_switch");

        std::cout << "stopping process periodic_cloud" << std::endl;
        system("pkill -STOP periodic_cloud");
    }

}

void ROSAction::startAllOtherProcesses()
{
    if(Name!="base_torso_head_init_server")
    {
        std::string process_name = "base_torso_head";
        std::cout << "starting process " << process_name << std::endl;
        system((std::string("pkill -CONT ") + process_name).c_str());
    }

    if(Name!="amazon_detector")
    {
        std::string process_name = "detector";
        std::cout << "starting process " << process_name << std::endl;
        system((std::string("pkill -CONT ") + process_name).c_str());
    }

    if(Name!="pregrasp_server")
    {
        std::string process_name = "pregrasp";
        std::cout << "starting process " << process_name << std::endl;
        system((std::string("pkill -CONT ") + process_name).c_str());
    }

    if(Name!="grasp_object")
    {
        std::string process_name = "grasping";
        std::cout << "starting process " << process_name << std::endl;
        system((std::string("pkill -CONT ") + process_name).c_str());
    }

    if(Name!="drop_object")
    {
        std::string process_name = "dropping";
        std::cout << "starting process " << process_name << std::endl;
        system((std::string("pkill -CONT ") + process_name).c_str());
    }

//    {
//        std::cout << "starting process rviz" << std::endl;
//        system("pkill -CONT rviz");
//    }

//    std::cout << "starting process rqt_amazon" << std::endl;
//    system("pkill -CONT rqt_amazon");

}

