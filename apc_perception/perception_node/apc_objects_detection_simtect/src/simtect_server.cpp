/*
 *
 * Sergio Caccamo (caccamo@kth.se) - KTH Royal Institute of Technology
 * Rares Ambrus <raambrus@kth.se>  - KTH Royal Institute of Technology
 *
 * Development framework for APC 2016 - Action server lookforobject simtect
 * V 1.0 (under development) June-2016
 *
 * Description: The LookForObjectTecture class creates an actionlib server that
 aknoledges if a given
 * set of objects is detected by simtect (modified simtrack) check the tf position and publishes the
 correspondig tf
 *
 *
 *  Copyright (c) 2016, Sergio Caccamo, Rares Ambrus, CVAP, KTH
 *    All rights reserved.
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are
 met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of KTH nor the
 *         names of its contributors may be used to endorse or promote products
 *          derived from this software without specific prior written
 permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 IS" AND
 *    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 *
 *TODO :        To do
 *TOCHK:        To check
 *TOCNG:        To change
 *TBR:          To be removed
 *
 */

#include <apc_objects_detection_simtect/simtect_server.h>




LookForObjectSimtect::LookForObjectSimtect(std::string name) : as_(nh_, name, false), action_name_(name)
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&LookForObjectSimtect::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&LookForObjectSimtect::preemptCB, this));

    serviceClient_camera_switch = nh_.serviceClient<perception_services::SwitchCameraByName>("/feature_segmentation/switch_camera");
    serviceClient_objects_switch = nh_.serviceClient<perception_services::SwitchObjects>("/feature_segmentation/switch_objects");
    serviceClient_stop_tracking = nh_.serviceClient<perception_services::StopTracking>("/feature_segmentation/stop_tracking");


    //init analysis variables

    nh_.param<bool>("/debug", debug_, false);
    nh_.param<std::string>("/camera_topic_debug", camera_id_, "kinect_chest");

    if (debug_)
        std::cout<<"-- DEBUG MODE --( "<<camera_id_<<")"<<std::endl;

    //tf transmission flag and global settings
    starttransm_ = false;
    tf_initialized_ = false;
    tfListener = new (tf::TransformListener);
    cameraID_current_ = "";
    result_.side = "undefined";

    //get the vertical displacement constant
    getParam("/apc/perception/object_displacement_threshold", +0.03, z_displacement_compensation);
    getParam("/apc/perception/object_displacement_compensation", -0.04, z_displacement_threshold);

    as_.start();
}


LookForObjectSimtect::~LookForObjectSimtect(void) {
    ROS_INFO("Simtect server is shutting down");
}

template<typename T>
void LookForObjectSimtect::getParam(std::string name, T def, T &param)
{
  if (nh_.hasParam(name.c_str()))
  {
    nh_.getParam(name.c_str(), param);
  }
  else
  {
    param = def;
  }
}

void LookForObjectSimtect::goalCB() {
    // accept the new goal
    new_goal_ = *as_.acceptNewGoal();
    goal_ = new_goal_.goalID;
    result_.segmentationID = 00;


    // retrieve settings
    targetObj_ = new_goal_.targetObj;
    objectslist_ = new_goal_.objectslist;
    cameraID_ = new_goal_.cameraID;
    methodID_ = new_goal_.methodID;
    taskID_ = new_goal_.taskID;
    binID_ = new_goal_.binID;

    object_found_ = false;
    starttransm_ = false;
    tf_initialized_ = false;
    result_.side = "undefined";

    watchdog_ = 50;

    // check if is a stop transmission message
    if(goal_ == STOP_TRANSMISSION){
        // stop tracking
        //perception_services::StopTracking service_request_stop_tracking;
        //bool succeded = serviceClient_stop_tracking.call(service_request_stop_tracking);
        return;
    }

    ROS_INFO("Simtect: Received new goal (%d)", goal_);
    ROS_INFO("Simtect: Looking for %s on %s", targetObj_.c_str(), binID_.c_str());
    ROS_INFO("Simtect: Using camera %s", cameraID_.c_str());

    // Simtect method
    perception_services::SwitchCameraByName service_request_switch_camera;
    service_request_switch_camera.request.camera = cameraID_;
    bool succeded = serviceClient_camera_switch.call(service_request_switch_camera);
    if (!succeded){
        as_.setAborted(result_);
        return;
    }

    std::vector<std::string> tempObjList(1);
    tempObjList[0]= targetObj_;
    //double model for some object
    if (targetObj_ == "oral_b_toothbrush_green" || targetObj_ == "oral_b_toothbrush_red" || targetObj_ == "safety_first_outlet_plugs"  ){
        tempObjList.push_back(targetObj_+"_bottom");
    }


    perception_services::SwitchObjects service_request_switch_objects;
    service_request_switch_objects.request.model_names = tempObjList;
    succeded = serviceClient_objects_switch.call(service_request_switch_objects);
    if (!succeded){
        as_.setAborted(result_);
        return;
    }

    // unregister any subscribed topic
    sub_.shutdown();

    // subscribe to simtect to check the desired object (list) and publish tfs
    std::string topicname_ = "feature_segmentation/" + targetObj_;
    sub_ = nh_.subscribe(topicname_, 1, &LookForObjectSimtect::analysisCB, this);

}

/*
   *  Callback for preempt actions
  */
void LookForObjectSimtect::preemptCB() {
    ROS_INFO("Simtect: Preempted (%s)", action_name_.c_str());
    //stop transmission
    starttransm_ = false;

    // unregister any subscribed topic
    sub_.shutdown();

    // stop tracking
    perception_services::StopTracking service_request_stop_tracking;
    bool succeded = serviceClient_stop_tracking.call(service_request_stop_tracking);

    // set the action state to preempted
    as_.setPreempted();
}

void LookForObjectSimtect::analysisCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {


    // make sure that the action hasn't been canceled
    if (!as_.isActive()){
        // unregister any subscribed topic
        sub_.shutdown();
        return;
    }


    if(!_checkTfPosition()){
        as_.setAborted(result_);
        ROS_INFO("Simtect: object detected in the wrong bin");
        return;
    }

    // stop tracking
    perception_services::StopTracking service_request_stop_tracking;
    bool succeded = serviceClient_stop_tracking.call(service_request_stop_tracking);

    object_found_ = true;
    starttransm_ = true;
    objectpose_ = *msg;



    ROS_INFO("Simtect: Succeeded, object(%s) found", targetObj_.c_str());

    // set the action state to succeeded
    as_.setSucceeded(result_);



}



/*
   *  stop the code until the system is available :debug
   */
bool LookForObjectSimtect::waitforGeneric (int TIMER){
    int timer_init = TIMER;
    //std::cout <<"\tD-Waiting - "<<std::flush;
    while( TIMER-- > 0 ){
        usleep(1000000);
        //cout << "(" << TIMER <<")-"<<std::flush;
    }
    //std::cout <<"ready"<<std::endl;
    return true;
}

/*
   *  this method broadcasts the last detected pose of the target object as tf
  */
void LookForObjectSimtect::threaded_broadcaster_object() {
    // broadcast the tf if the object has been found
    if (starttransm_) {
        std::string objectname_ = targetObj_ + "_final";
        std::string cameraframe_ = objectpose_.header.frame_id;
        transform.setOrigin(tf::Vector3(objectpose_.pose.position.x,
                                        objectpose_.pose.position.y,
                                        objectpose_.pose.position.z ));
        transform.setRotation(tf::Quaternion(
                                  objectpose_.pose.orientation.x, objectpose_.pose.orientation.y,
                                  objectpose_.pose.orientation.z, objectpose_.pose.orientation.w));


        //debug_

        //if the camera frame is not fixed, tranform w.r.t. base
        if(!tf_initialized_){
            tf::StampedTransform tr;
            try{
                listener.waitForTransform("base", cameraframe_, ros::Time::now(), ros::Duration(1.0));
                listener.lookupTransform("base", cameraframe_, ros::Time(0), tr);
                tf_initialized_ = true;
                from_ids_transform = tr * transform;
                from_ids_transform.setOrigin(tf::Vector3( from_ids_transform.getOrigin().x(),
                                                          from_ids_transform.getOrigin().y(),
                                                          from_ids_transform.getOrigin().z() + shift_height_tf));
                br.sendTransform(tf::StampedTransform(from_ids_transform, ros::Time::now(), "base", objectname_));
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s: base tf to ids not found",ex.what());
                //send whatever we have for now
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), cameraframe_, objectname_));
            }
        }
        else{
            br.sendTransform(tf::StampedTransform(from_ids_transform, ros::Time::now(), "base", objectname_));
        }

    }
}

/*
   *  this method checks if the detected objects is inside the desired bin
  */
bool LookForObjectSimtect::_checkTfPosition(){
    //check if the detected object is in the target bin
    tf::StampedTransform tr;
    try{
        listener.waitForTransform(binID_, targetObj_, ros::Time::now(), ros::Duration(1.0));
        ROS_INFO_STREAM("Waited for transform");
        listener.lookupTransform(binID_, targetObj_, ros::Time(0), tr);
        ROS_INFO_STREAM("Found for transform");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s: bin tf to object not found",ex.what());
        return false;
    }

    //z coordinate should always be close to the bin ground
    if (tr.getOrigin().z() < z_displacement_threshold){
        ROS_ERROR("SIMTECT: The vertical displacement of the detected object is too high");
        return false;
    }
    //TBR fix eith of the bin
    shift_height_tf = 0.0;
    if (tr.getOrigin().z() < 0.02){
        ROS_WARN("SIMTECT: Had to adjust the z value of the object");
        shift_height_tf = z_displacement_compensation;
    }

    //check if the object is outside the target bin

    switch (binID_.at(binID_.size()-1)){
    case 'A': case 'C': case 'J': case 'L':
        if((tr.getOrigin().z() > 0.265) || (fabs(tr.getOrigin().y()) > 0.135) || (tr.getOrigin().x() < -0.01) || (tr.getOrigin().x() > 0.43)){
            return false;
        }

        if(tr.getOrigin().y() <= 0)
            result_.side = "right";
        else
            result_.side = "left";

        break;
    case 'B': case 'K':
        if((tr.getOrigin().z() > 0.265) || (fabs(tr.getOrigin().y()) > 0.15) || (tr.getOrigin().x() < -0.01) || (tr.getOrigin().x() > 0.43)){
            return false;
        }

        if(tr.getOrigin().y() <= 0)
            result_.side = "right";
        else
            result_.side = "left";

        break;
    case 'D': case 'G': case 'F': case 'I':
        if((tr.getOrigin().z() > 0.225) || (fabs(tr.getOrigin().y()) > 0.135) || (tr.getOrigin().x() < -0.01) || (tr.getOrigin().x() > 0.43)){
            return false;
        }

        if(tr.getOrigin().y() <= 0)
            result_.side = "right";
        else
            result_.side = "left";

        break;
    case 'E': case 'H':
        if((tr.getOrigin().z() > 0.225) || (fabs(tr.getOrigin().y()) > 0.15) || (tr.getOrigin().x() < -0.01) || (tr.getOrigin().x() > 0.43)){
            return false;
        }

        if(tr.getOrigin().y() <= 0)
            result_.side = "right";
        else
            result_.side = "left";

        break;
    case 'e': //tote
        if(fabs((tr.getOrigin().x()) > 0.28) || (fabs(tr.getOrigin().y()) > 0.18)){
            ROS_ERROR("SIMTECT: Detected object is outside the tote bounds");
            return false;
        }

        if(tr.getOrigin().x() >= 0)
            result_.side = "right";
        else
            result_.side = "left";

        break;
    }

    return true;
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "lookforobject_simtect");

    LookForObjectSimtect LookForObjectSimtect(ros::this_node::getName());

    ROS_INFO("Simtect Action server running");
    // ros::spin(); //single thread implementation

    // ros::AsyncSpinner spinner(4);  // Use 4 threads
    ros::Rate r(10); // 10 hz
    // spinner.start();

    while (ros::ok()) {
        r.sleep();
        LookForObjectSimtect.threaded_broadcaster_object();
        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}
