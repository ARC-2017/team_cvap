/*
 *
 * Sergio Caccamo (caccamo@kth.se) - KTH Royal Institute of Technology
 *
 * Development framework for APC 2016 - Action server lookforobject
 * V 1.0 (under development) April-2016
 *
 * Description: The LookForObject class creates an actionlib server that
 aknoledges if a given
 * set of objects is detected by the perception nodes and publishes the
 correspondig tf
 *
 *
 *  Copyright (c) 2016, Sergio Caccamo, CVAP, KTH
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

#include <apc_objects_detection_action/apc_objects_detection_server.h>




LookForObject::LookForObject(std::string name) : as_(nh_, name, false), action_name_(name)
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&LookForObject::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&LookForObject::preemptCB, this));

    ROS_INFO_STREAM("Initializing texture ActionServer");
    ac_texture_ = boost::shared_ptr<actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> >(new actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction>("lookforobject_texture")) ;
    ac_texture_->waitForServer();
    ROS_INFO_STREAM("Texture ActionServer initialized");

    ROS_INFO_STREAM("Initializing RGBD ActionServer");
    ac_rgbd_ = boost::shared_ptr<actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> >(new actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction>("lookforobject_rgbd")) ;
    ac_rgbd_->waitForServer();
    ROS_INFO_STREAM("RGBD ActionServer initialized");

    ROS_INFO_STREAM("Initializing simtect ActionServer");
    ac_simtect_ = boost::shared_ptr<actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction> >(new actionlib::SimpleActionClient<apc_objects_detection_action::LookForObjectAction>("lookforobject_simtect")) ;
    ac_simtect_->waitForServer();
    ROS_INFO_STREAM("Simtect ActionServer initialized");

    //init analysis variables
    nh_.param<bool>("/debug", debug_, false);
    nh_.param<bool>("/baxter_face_ctrl", baxter_face_ctrl_, true);


    // Face for baxter! :)
    if (baxter_face_ctrl_) {
        face_pub_= nh_.advertise<std_msgs::Float32>("/robot/expressions/selector", 1);
        face_bg_pub_= nh_.advertise<std_msgs::Float32>("/robot/background/selector", 1);
    }

    as_.start();

    waitforGeneric(1);
    //change face on baxter (sweet)
    change_face(BKG_TEXTURE,FACE_SWEET);

}


LookForObject::~LookForObject(void) {
    ROS_INFO("Perception server is shutting down");
}




void LookForObject::goalCB() {
    // accept the new goal
    /**/

    // cancell all transmition
    apc_objects_detection_action::LookForObjectGoal goal_stop_transmission;
    goal_stop_transmission.goalID = STOP_TRANSMISSION;
    ac_rgbd_->sendGoal(goal_stop_transmission);
    ac_texture_->sendGoal(goal_stop_transmission);
    ac_simtect_->sendGoal(goal_stop_transmission);
    feedback_.info = "Preempting";
    as_.publishFeedback(feedback_);
    waitforGeneric(1); // 1 second

    if (as_.isActive()){
        // we are currently executing another goal -> preempt it first
        as_.setPreempted();
        // wait for the goal to be preempted

    }


    new_goal_ = *as_.acceptNewGoal();
    goal_ = new_goal_.goalID;

    // retrieve settings
    targetObj_ = new_goal_.targetObj;
    objectslist_ = new_goal_.objectslist;
    cameraID_ = new_goal_.cameraID;
    methodID_ = new_goal_.methodID;
    taskID_ = new_goal_.taskID;
    binID_ = new_goal_.binID;
    globalSearch_ = new_goal_.globalSearch;

    ROS_INFO("Received new goal [OBJ: %s BIN: %s]", targetObj_.c_str(),binID_.c_str());
    // ROS_INFO("Looking for %s on %s", targetObj_.c_str(), binID_.c_str());

    // reset helper variables
    int method_segment = lookForObjectKind(targetObj_,cameraID_,methodID_,objectslist_, globalSearch_);
    bool finished_before_timeout = false;
    std::string state = "";

    switch (method_segment){


    case TEXTURE_SEGMENT:
    {
        ROS_INFO("Texture");
        //change face on baxter (focused)
        if (!globalSearch_)
            change_face(BKG_YOLO,FACE_SWEAT);
        else
            change_face(BKG_YOLO,FACE_SWEAT);

        feedback_.info = "TEXTURE";
        as_.publishFeedback(feedback_);

        ac_texture_->sendGoal(new_goal_);
        // wait for the action to return
        for (size_t i=0; i<(TIME_OUT_SENS * 3); i++){
            if (as_.isPreemptRequested()){
                ac_texture_->cancelAllGoals();
                return;
            }

            finished_before_timeout = ac_texture_->waitForResult(ros::Duration(0.3));
            if (finished_before_timeout){
                break;
            }
            ros::spinOnce();
        }

        result_= *(ac_texture_->getResult());
        state = ac_texture_->getState().toString();
        result_.trueTF = true;



    }; break;

    case SIMTECT_SEGMENT:
    {
        ROS_INFO("Simtect");

        //change face on baxter (focused)
        change_face(BKG_SIMTECT,FACE_FOCUSED);


        feedback_.info = "SIMTECT";
        as_.publishFeedback(feedback_);

        ac_simtect_->sendGoal(new_goal_);
        // wait for the action to return
        for (size_t i=0; i<(TIME_OUT_SENS * 3); i++){
            if (as_.isPreemptRequested()){
                ac_simtect_->cancelAllGoals();
                return;
            }

            finished_before_timeout = ac_simtect_->waitForResult(ros::Duration(0.3));
            if (finished_before_timeout){
                break;
            }
            ros::spinOnce();
        }

        result_= *(ac_simtect_->getResult());
        state = ac_simtect_->getState().toString();
        result_.trueTF = true;

    }; break;


    case RGBD_SEGMENT:
    {
        ROS_INFO("RGBD");
        change_face(BKG_RGBD,FACE_FOCUSED);	//change face on baxter (focused)
        feedback_.info = "RGBD";
        as_.publishFeedback(feedback_);

        //ac_rgbd_->waitForServer(); //TOCHK
        ac_rgbd_->sendGoal(new_goal_);

        for (size_t i=0; i<(TIME_OUT_SENS * 3); i++){
            if (as_.isPreemptRequested()){
                ac_rgbd_->cancelAllGoals();
                return;
            }

            finished_before_timeout = ac_rgbd_->waitForResult(ros::Duration(0.3));
            if (finished_before_timeout){
                break;
            }
            ros::spinOnce();
        }

        result_= *(ac_rgbd_->getResult());
        state = ac_rgbd_->getState().toString();
        result_.trueTF = false;

    }; break;

    default:{
        ROS_INFO("The target object cannot be detected by the perception server");
    }; break;
    }

    result_.segmentationID = method_segment;

    if (finished_before_timeout && !as_.isPreemptRequested() && as_.isActive())
    {

        ROS_INFO("Action finished: %s",state.c_str());
        if (state == "SUCCEEDED"){
            //change face on baxter (happy)
            change_face(BKG_CURRENT,FACE_HAPPY);

            // set the action state to succeeded
            as_.setSucceeded(result_);
        } else {
            //change face on baxter (sad)
            change_face(BKG_CURRENT,FACE_SAD);

            // set the action state to aborted
            as_.setAborted(result_);
        }
    }
    else{
        //change face on baxter (sweet)
        change_face(BKG_CURRENT,FACE_SAD);

        ROS_INFO("Action did not finish before the time out.");
        as_.setAborted(result_);
    }


}


/*
   *  Callback for defining the segmentation method (object oriented)
   *
*/
int LookForObject::lookForObjectKind(std::string object, std::string cameraid, std::string force_rgbd, std::vector<std::string>& non_target_objects, bool globalsearch){

    // exeptions
    if((object == "oral_b_toothbrush_green") || (object == "oral_b_toothbrush_red")){
                    for(size_t i=0; i<non_target_objects.size(); i++){
                        if(non_target_objects[i]=="oral_b_toothbrush_red" || non_target_objects[i]=="oral_b_toothbrush_green")
                              return NO_SEGMENT;
                    }
    }

    // IDS RASHTECT
    if (cameraid.find("ids")!=std::string::npos && !globalsearch ){
        if ( object == "womens_knit_gloves" || object == "cherokee_easy_tee_shirt" || object == "cloud_b_plush_bear" ||
             object == "fitness_gear_3lb_dumbbell" || object == "easter_trurtle_sippy_cup" || object == "fiskars_scissors_red"  ){
            return NO_SEGMENT;
        }
        return SIMTECT_SEGMENT;

    }

    //RGBD analysis
    else {
        if (    object == "womens_knit_gloves" || object == "cherokee_easy_tee_shirt" || (object == "scotch_bubble_mailer") ||
                object == "cloud_b_plush_bear" || (object == "kleenex_tissue_box" && force_rgbd.find("rgbd")!= std::string::npos) ||
                object == "kleenex_paper_towels" || object == "hanes_tube_socks" || object == "peva_shower_curtain_liner" ||
                object == "staples_index_cards"
                ){

            //blue
            if(object == "kleenex_tissue_box"){ //target object is not in the non target objects list


                bool no_conflict=true;
                for(size_t i=0; i<non_target_objects.size(); i++){
                    if(non_target_objects[i]=="clorox_utility_brush"||
                            non_target_objects[i]=="dr_browns_bottle_brush"||
                            non_target_objects[i]=="dasani_water_bottle"||
                            non_target_objects[i]=="expo_dry_erase_board_eraser"||
                            non_target_objects[i]=="cool_shot_glue_sticks"||
                            non_target_objects[i]=="peva_shower_curtain_liner"||
                            non_target_objects[i]=="i_am_a_bunny_book"||
                            non_target_objects[i].find("oral_b_toothbrush")!= std::string::npos||
                            non_target_objects[i]=="fitness_gear_3lb_dumbbell"||
                            non_target_objects[i]=="kyjen_squeakin_eggs_plush_puppies"||
                            non_target_objects[i]=="kleenex_tissue_box"){
                        no_conflict=false;
                        break;
                    }
                }
                if(no_conflict){
                    return RGBD_SEGMENT;
                }
                else{
                    return TEXTURE_SEGMENT;
                }
            }
            //purple //TBCHK
            if(object == "staples_index_cards"){ //target object is not in the non target objects list


                bool no_conflict=true;
                for(size_t i=0; i<non_target_objects.size(); i++){
                    if(non_target_objects[i]=="kleenex_paper_towels"){
                        no_conflict=false;
                        break;
                    }
                }
                if(no_conflict){
                    return RGBD_SEGMENT;
                }
                else{
                    return TEXTURE_SEGMENT;
                }
            }
            //yellow
            else if (object == "scotch_bubble_mailer" || object == "cloud_b_plush_bear"  ){
                bool no_conflict=true;
                for(size_t i=0; i<non_target_objects.size(); i++){
                    if(non_target_objects[i]=="crayola_24_ct"||
                            non_target_objects[i]=="kyjen_squeakin_eggs_plush_puppies"||
                            //non_target_objects[i]=="scotch_bubble_mailer"||
                            non_target_objects[i]=="safety_first_outlet_plugs"||
                            non_target_objects[i]=="ticonderoga_12_pencils"||
                            //non_target_objects[i]=="cloud_b_plush_bear" ||
                            (non_target_objects[i]=="jane_eyre_dvd" && object == "cloud_b_plush_bear") ||
                            non_target_objects[i]=="command_hooks"){
                        no_conflict=false;
                        break;
                    }
                }
                if(no_conflict){
                    return RGBD_SEGMENT;
                }
                else if(object == "cloud_b_plush_bear"){
                    return NO_SEGMENT;
                }
                else {
                    return TEXTURE_SEGMENT;
                }

            }
            //white
            else if((object == "kleenex_paper_towels") || (object == "hanes_tube_socks") || (object == "cherokee_easy_tee_shirt") || (object == "peva_shower_curtain_liner")){
                bool no_conflict=true;
                for(size_t i=0; i<non_target_objects.size(); i++){
                    if(non_target_objects[i]=="kleenex_paper_towels"||
                            non_target_objects[i]=="hanes_tube_socks"||
                            non_target_objects[i]=="up_glucose_bottle"||
                            non_target_objects[i]=="cherokee_easy_tee_shirt"||
                            non_target_objects[i]=="elmers_washable_no_run_school_glue"||
                            non_target_objects[i]=="dr_browns_bottle_brush"||
                            non_target_objects[i]=="clorox_utility_brush"||
                            non_target_objects[i]=="soft_white_lightbulb"||
                            non_target_objects[i]=="barkely_hide_bones"||
                            non_target_objects[i]=="command_hooks"||
                            non_target_objects[i]=="staples_index_cards"||
                            non_target_objects[i]=="cool_shot_glue_sticks"||
                            non_target_objects[i]=="peva_shower_curtain_liner"||
                            non_target_objects[i]=="dove_beauty_bar"||
                            non_target_objects[i]=="woods_extension_cord"||
                            non_target_objects[i]=="barkely_hide_bones"||
                            non_target_objects[i]=="rawlings_baseball"||
                            non_target_objects[i]=="command_hooks"){
                        no_conflict=false;
                        break;
                    }
                }
                if(no_conflict){
                    return RGBD_SEGMENT;
                }
                else if (object == "cherokee_easy_tee_shirt"){
                    return RGBD_SEGMENT;
                }
                else{
                    return TEXTURE_SEGMENT;
                }
            }
            //black
            else if (object == "womens_knit_gloves"){
                bool no_conflict=true;
                for(size_t i=0; i<non_target_objects.size(); i++){
                    if(non_target_objects[i]=="rolodex_jumbo_pencil_cup"){
                        no_conflict=false;
                        break;
                    }
                }
                if(no_conflict){
                    return RGBD_SEGMENT;
                }
                else{
                    return NO_SEGMENT;
                }
            }
            //green not tested yet
            else
                return RGBD_SEGMENT;
        }
        else {
            return TEXTURE_SEGMENT;
        }
    }
}

/*
   *  Callback for preempt actions
  */
void LookForObject::preemptCB() {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();

    // change face on baxter (sad)
    change_face(BKG_TEXTURE,FACE_SWEAT);
}



/*
   *  this method is completely useless and makes the code less readable. But I like it.
   *  if bkg <0 only the face will be changed
  */
void LookForObject::change_face(float bkg, float face_id)
{
    std_msgs::Float32 msg_face_;
    msg_face_.data =  face_id;
    //return if no control of baxter is allowed
    if (!baxter_face_ctrl_)
        return;

    face_pub_.publish(msg_face_);
    if (bkg >= 0){
        waitforGeneric(0.3);
        msg_face_.data =  bkg;
        face_bg_pub_.publish(msg_face_);
    }
}

/*
   *  stop the code until the system is available (sec) :debug
   */
bool LookForObject::waitforGeneric (float TIMER){
    int timer_init = TIMER * 10;
    //std::cout <<"\tD-Waiting - "<<std::flush;
    while( timer_init-- > 0 ){
        usleep(100000);
        //cout << "(" << TIMER <<")-"<<std::flush;
    }
    //std::cout <<"ready"<<std::endl;
    return true;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "lookforobject");

    LookForObject lookforobject(ros::this_node::getName());

    ROS_INFO("LookForObject Action server running");
    // ros::spin(); //single thread implementation

    //    ros::AsyncSpinner spinner(2);  // Use 4 threads
    ros::Rate r(10); // 10 hz
    //    spinner.start();

    while (ros::ok()) {
        r.sleep();
        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}
