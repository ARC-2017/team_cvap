#ifndef __APC_OBJECTS_DETECION_SERVER__
#define __APC_OBJECTS_DETECION_SERVER__


/*
 *
 * Sergio Caccamo (caccamo@kth.se) - KTH Royal Institute of Technology
 *
 * Development framework for APC 2016 - Object detection Server
 * V 1.0 (under development) May-2016
 *
 * Description: The LookForObject class creates an actionlib server that uses simtrack and RGBD segmentation
 * techniques for identify target objects and publish TFs
 *
 *  Copyright (c) 2016, Sergio Caccamo, CVAP, KTH
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
#include <simtrack_nodes/SwitchCameraByName.h>
#include <simtrack_nodes/SwitchObjects.h>

//Math
//#include <Eigen/Dense>

//Actions and threading
#include <apc_objects_detection_action/LookForObjectAction.h>
#include <boost/thread.hpp>

//watchdog timer
#define TIME_OUT_SENS 	20
#define STOP_TRANSMISSION -1

//segmentation methods
#define NO_SEGMENT		-1
#define TEXTURE_SEGMENT 0
#define SIMTECT_SEGMENT 1
#define RGBD_SEGMENT	2
#define PC_SEGMENT		3

#define FACE_HAPPY 		0
#define FACE_SAD		1
#define FACE_SWEET		2
#define FACE_CRAZY 		3
#define FACE_SLEEPY             4
#define FACE_FOCUSED            5
#define FACE_DOUBTFUL           6
#define FACE_SWEAT              7


#define BKG_CURRENT		-1
#define BKG_TEXTURE		0
#define BKG_SIMTECT		1
#define BKG_RGBD 		2
#define BKG_YOLO 		4



//Debug
#include <ctime>



class LookForObject
{
public:
    LookForObject(std::string name);
    ~LookForObject();

    void goalCB();
    void preemptCB();
    int lookForObjectKind(std::string object, std::string cameraid, std::string force_rgbd, std::vector<std::string>& non_target_objects,bool globalsearch);

    void change_face(float bkg, float face_id);
    bool waitforGeneric (float TIMER);

private:

protected:
    ros::NodeHandle nh_;

    geometry_msgs::PoseStamped objectpose_;
    actionlib::SimpleActionServer<apc_objects_detection_action::LookForObjectAction> as_;
    boost::shared_ptr<actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> > ac_texture_;
    boost::shared_ptr<actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> > ac_rgbd_;
    boost::shared_ptr<actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> > ac_simtect_;

    //simtrack attributes
    ros::ServiceClient serviceClient_camera_switch;
    ros::ServiceClient serviceClient_objects_switch;

    //analysis variables
    std::string quality_;
    std::string camera_id_;
    bool debug_,baxter_face_ctrl_;

    // perception settings and action_msgs
    std::string action_name_;
    std_msgs::String msg_str;
    std::string targetObj_, param_;
    std::vector<std::string> objectslist_, objectslistglobal_;
    std::string cameraID_, methodID_, taskID_, binID_;
    bool globalSearch_;
    int goal_, watchdog_;
    bool object_found_, starttransm_;


    apc_objects_detection_action::LookForObjectFeedback feedback_;
    apc_objects_detection_action::LookForObjectResult result_;
    apc_objects_detection_action::LookForObjectGoal new_goal_;
    ros::Subscriber sub_;
    ros::Publisher face_pub_,face_bg_pub_;
    

    // TF transmission related
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::TransformListener* tfListener;
    tf::TransformListener listener;
    tf::StampedTransform camera_to_bin;




};


#endif // __APC_OBJECTS_DETECION_SERVER__
