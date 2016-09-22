#ifndef __SIMTECT_SERVER__
#define __SIMTECT_SERVER__

/*
 *
 * Sergio Caccamo (caccamo@kth.se) - KTH Royal Institute of Technology
 *
 * Development framework for APC 2016 - Object detection Server Simtect
 * V 1.0 (under development) June-2016
 *
 * Description: The LookForObject class creates an actionlib server that uses simtrack and RGBD segmentation
 * techniques for identify target objects and publish TFs
 *
 *  Copyright (c) 2016, Sergio Caccamo, Rares Ambrus, CVAP, KTH
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

//System
#include <iostream>
#include <string.h>
#include <vector>
//ROS
#include <ros/ros.h>

//Messages
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>


//simtrack
#include <perception_services/SwitchCameraByName.h>
#include <perception_services/SwitchObjects.h>
#include <perception_services/StopTracking.h>

//Math
#include <Eigen/Dense>

//Actions and threading
#include <apc_objects_detection_action/LookForObjectAction.h>
#include <boost/thread.hpp>

//watchdog timer
#define TIME_OUT_SENS 20
#define STOP_TRANSMISSION -1


//Debug
#include <ctime>



class LookForObjectSimtect
{
public:
    LookForObjectSimtect(std::string name);
    ~LookForObjectSimtect();

    void goalCB();
    void preemptCB();
    void analysisCB(const geometry_msgs::PoseStamped::ConstPtr &msg);

    bool _checkTfPosition();
    std::string setCameraTopic( std::string camera);
    void threaded_broadcaster_object();
    bool waitforGeneric (int TIMER);

private:

protected:
    ros::NodeHandle nh_;

    geometry_msgs::PoseStamped objectpose_;
    actionlib::SimpleActionServer<apc_objects_detection_action::LookForObjectAction> as_;

    //simtect attributes
    ros::ServiceClient serviceClient_camera_switch;
    ros::ServiceClient serviceClient_objects_switch;
    ros::ServiceClient serviceClient_stop_tracking;

    //analysis variables
    std::string quality_;
    std::string camera_id_;
    bool debug_;

    // perception settings and action_msgs
    std::string action_name_;
    std_msgs::String msg_str;
    std::vector<std::string> objectslist_;
    std::string targetObj_, param_;
    std::string cameraID_, cameraID_current_, methodID_, taskID_, binID_;
    int goal_, watchdog_;
    bool object_found_, starttransm_, tf_initialized_;
    double shift_height_tf;
    double z_displacement_compensation, z_displacement_threshold;


    apc_objects_detection_action::LookForObjectFeedback feedback_;
    apc_objects_detection_action::LookForObjectResult result_;
    apc_objects_detection_action::LookForObjectGoal new_goal_;
    ros::Subscriber sub_;

    // TF transmission related
    tf::TransformBroadcaster br;
    tf::Transform transform, from_ids_transform;
    tf::TransformListener* tfListener;
    tf::TransformListener listener;
    tf::StampedTransform camera_to_bin;

    template<typename T>
    void getParam(std::string name, T def, T &param);


};


#endif // __SIMTECT_SERVER__
