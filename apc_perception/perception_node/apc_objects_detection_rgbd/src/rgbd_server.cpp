/*
 *
 * Sergio Caccamo (caccamo@kth.se) - KTH Royal Institute of Technology
 *
 * Development framework for APC 2016 - Object detection RGBD
 * V 1.0 (under development) May-2016
 *
 * Description: The LookForObjectRGBD class creates an actionlib server thatuses RGBD segmentation
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


#include <apc_objects_detection_rgbd/rgbd_server.h>



/* Logic explaination:
 *
 * after a goal request, the server set up all the search variables specific for a target object
 * then it subscribes to the kinect2/openni2 topic and start the segmentation process for 10 iterations:
 * If nothing has been found in 10 iterations then the system returns failed.
 *
 *Carefull: what happens if the RGBD sensor didn't start? Nothing. Idiot.
 */


LookForObjectRGBD::LookForObjectRGBD(std::string name) : as_(nh_, name, false), action_name_(name)
{
    // register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&LookForObjectRGBD::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&LookForObjectRGBD::preemptCB, this));

    //init analysis variables
    nh_.param<bool>("/debug", debug_, false);
    nh_.param<std::string>("/camera_topic_debug", camera_topic_, "kinect2_cheast");
    nh_.param<std::string>("/quality", quality_, "sd");
    nh_.param<bool>("/publish_output_pc", publish_output_pc_, true);


    if (debug_)
        std::cout<<"-- DEBUG MODE --( "<<camera_topic_<<")"<<std::endl;

    if (publish_output_pc_){
        std::cout<<"-- OUTPUT ON --( /pc_RGB_output )"<<std::endl;
        this->pub_output = nh_.advertise<sensor_msgs::PointCloud2> ("pc_RGB_filtered", 1);
    }

    //tf transmission flag
    starttransm_ = false;
    tfListener = new (tf::TransformListener);
    result_.side = "undefined";
    
    
    //setting PCL - debug mode
    x_roi_min =  - 0.2;//0.22;           //Define the ROI xmin
    x_roi_max =  + 0.2;//0.7;            //Define the ROI xmax
    z_roi_min =  - 1.5;//0.2;            //Define the ROI zmin
    z_roi_max =  + 1.5;//0.6;            //Define the ROI zmax
    y_roi_min =  - 0.05;//0.0;            //Define the ROI ymin
    y_roi_max =  + 0.55;//0.5;            //Define the ROI ymax

    //cloud = new pcl::PCLPointCloud2();
    //cloud_filtered_ptr =  ( new pcl::PCLPointCloud2())->ConstPtr;
    //cloud_rgb = new pcl::PointCloud <pcl::PointXYZRGB>();
    //transformed_cloud_rgb = new pcl::PointCloud<pcl::PointXYZRGB>();         // initial point cloud

    as_.start();

}


LookForObjectRGBD::~LookForObjectRGBD(void)
{
    ROS_INFO("'RGBD server is shutting down");
}

void LookForObjectRGBD::goalCB()
{
    // accept the new goal
    new_goal_ = *as_.acceptNewGoal();
    goal_ = new_goal_.goalID;
        result_.segmentationID = 00;

    //retrieve settings
    targetObj_ = new_goal_.targetObj;
    objectslist_ = new_goal_.objectslist;
    cameraID_ = new_goal_.cameraID;
    methodID_ = new_goal_.methodID;
    taskID_ = new_goal_.taskID;
    binID_ = new_goal_.binID;


    // reset helper variables
    object_found_ = false;
    starttransm_ = false;
    result_.side = "undefined";
    watchdog_ = TIME_OUT_SENS;

    // check if is a stop transmission message
    if(goal_ == STOP_TRANSMISSION){
        return;
    }
    ROS_INFO("RGBD: Received new goal (%d)", goal_);

    // start analysis
    ROS_INFO("RGBD: Looking for %s on %s", targetObj_.c_str(), binID_.c_str());  //@TOCHK: change object
    // subscribe to the correct camera topic
    std::string topicname_ = this->setCameraTopic(cameraID_);
    sub_ = nh_.subscribe(topicname_, 1, &LookForObjectRGBD::analysisCB, this);

}


/*
   *  Callback for preempt actions
  */
void LookForObjectRGBD::preemptCB()
{
    ROS_INFO("RGBD: Preempted (%s)", action_name_.c_str());
    //stop transmission
     starttransm_ = false;
    // unregister any subscribed topic
    sub_.shutdown();
    // set the action state to preempted
    as_.setPreempted();
    
}

/*
   *  Callback for choosing the camera topic
   *  if in debug mode the algorithm will only look at the camera topic specified in the launch file
  */
std::string LookForObjectRGBD::setCameraTopic( std::string camera)
{
    //topic if in debug
    if(debug_){
        return camera_topic_;
    }
    //kinect2 like output
    return "/" + camera + "/" + quality_ + "/points";
}


/* POINT CLOUD subscribe
   * Callback that does the analysis of the pc and looks for target object
   */
void LookForObjectRGBD::analysisCB(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    geometry_msgs::PoseStamped msg_pose;
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
        return;

    // look for the object only for a limited number of frames
    watchdog_ --;
    std::cerr<<":"<<watchdog_<<"";


    // Analysis of PC
    // PCs
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr transformed_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr claster_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud <pcl::PointXYZHSV>::Ptr claster_cloud_hsv (new pcl::PointCloud<pcl::PointXYZHSV> ());


    // Container for original & unfiltered data
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    try{
        // reading tfs from bins
        tfListener->waitForTransform(binID_, cloud_msg->header.frame_id, ros::Time::now(), ros::Duration(0.0));
        tfListener->lookupTransform(binID_, cloud_msg->header.frame_id, ros::Time(0), camera_to_bin);

        // prepare the tf header for transmission
        msg_pose.header.frame_id = binID_;

        sensor_msgs::PointCloud2 cloud_rotated;
        //rotate in bin frame and set the crop ROI limits for the bin or the tote
        cloud_rotated = LookForObjectRGBD::set_bin_cloud(cloud_msg);

        // Convert to PCL data type
        pcl_conversions::toPCL(cloud_rotated, *cloud);

    }
    //If the bin tf doesn't exists work on camera frame
    catch (tf::TransformException ex){
        ROS_WARN("Bin TF not found: setting standard tf ");
        //ros::Duration(1.0).sleep();

        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_msg, *cloud);

        // prepare the tf header for transmission (no bin rotation )
        std::cout<< "Parent frame: "<<cloud->header.frame_id<<std::endl;
        msg_pose.header.frame_id = cloud->header.frame_id;
    }

    //reset orientation for object
    camera_to_bin.setOrigin(tf::Vector3(0,0,0));
    camera_to_bin.setRotation(tf::Quaternion(0,0,0,1)); //null rotation

    // Convert from PCLPointCloud2 to PointXYZRGB
    pcl::fromPCLPointCloud2 (*cloud, *cloud_rgb);

    // Select the portion of PC of interest (Bin content, tote, or simple roi for debug)
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_rgb);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (z_roi_min, z_roi_max);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*transformed_cloud_rgb);

    pass.setInputCloud (transformed_cloud_rgb);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (x_roi_min, x_roi_max);
    pass.filter (*transformed_cloud_rgb);
    pass.setInputCloud (transformed_cloud_rgb);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_roi_min, y_roi_max);
    pass.filter (*transformed_cloud_rgb);
    pass.filter (*indices);

    //  RGBD segmentation
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (transformed_cloud_rgb);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10); //6
    reg.setPointColorThreshold (6); //8
    reg.setRegionColorThreshold (5); //12
    reg.setMinClusterSize (500);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    //tf origin
    float centroid[] = {0.0 ,0.0 ,0.0};
    int idx = 0;
    // Analyzes single cluster from Segmentation
    for( std::vector <pcl::PointIndices>::iterator it = clusters.begin(); it!= clusters.end(); it++){
        //std::cerr<<":";
        pcl::PointIndices::Ptr inliers(new  pcl::PointIndices()); ;
        inliers->indices = (*it).indices;//(*it).Ptr;

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        // Extract the inliers
        extract.setInputCloud (transformed_cloud_rgb);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*claster_cloud_rgb);

        // function for analyzing the cluster

        float th_rgb[] = {130.0 ,130.0 ,130.0};
        float th_rgb_strong[] = {180.0 ,180.0 ,180.0};
        float rgb_feature[] = {0.0 ,0.0 ,0.0};
        float hsv_feature[] = {0.0 ,0.0 ,0.0};
        double hsv_sum_squared [] = {0.0 ,0.0 ,0.0};
        float hsv_std [] = {0.0 ,0.0 ,0.0};
        float rgb_feature_proportional[] = {0.0 ,0.0 ,0.0};
        float rgb_feature_predominant[] = {0.0 ,0.0 ,0.0};
        centroid[0] = 0.0; centroid[1] = 0.0; centroid[2] = 0.0;
        idx++;


        // features extraction
        for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = claster_cloud_rgb->begin(); it!= claster_cloud_rgb->end(); it++){
            pcl::PointXYZHSV pointHSV ;

            pcl::PointXYZRGBtoXYZHSV( *it , pointHSV);

            //HSV
            hsv_feature[H_] += pointHSV.h;
            hsv_feature[S_] += pointHSV.s;
            hsv_feature[V_] += pointHSV.v;
            hsv_sum_squared[H_] += (pointHSV.h * pointHSV.h);
            hsv_sum_squared[S_] += (pointHSV.s * pointHSV.s);
            hsv_sum_squared[V_] += (pointHSV.v * pointHSV.v);

            //Centroid
            centroid[0]+=it->x;
            centroid[1]+=it->y;
            centroid[2]+=it->z;

            rgb_feature[R_]+=it->r;
            rgb_feature[G_]+=it->g;
            rgb_feature[B_]+=it->b;
            if(it->r > th_rgb_strong[R_]){
                rgb_feature_proportional[R_]++;
            }
            if(it->g > th_rgb_strong[G_]){
                rgb_feature_proportional[G_]++;
            }
            if(it->b > th_rgb_strong[B_]){
                rgb_feature_proportional[B_]++;
            }
            if(it->r > th_rgb_strong[R_] && it->g < th_rgb_strong[G_] && it->b < th_rgb_strong[B_]){
                rgb_feature_predominant[R_]++;
            }
            if(it->g > th_rgb_strong[G_] && it->r < th_rgb_strong[R_] && it->b < th_rgb_strong[B_]){
                rgb_feature_predominant[G_]++;
            }
            if(it->b > th_rgb_strong[B_] && it->g < th_rgb_strong[G_] && it->r < th_rgb_strong[R_]){
                rgb_feature_predominant[B_]++;
            }
        }

        float N = claster_cloud_rgb->size();
        //centroid for tf
        centroid[0] = centroid[0] / N;
        centroid[1] = centroid[1] / N;
        centroid[2] = centroid[2] / N;
        //average of color
        rgb_feature[R_] = rgb_feature[R_] / N;
        rgb_feature[G_] = rgb_feature[G_] / N;
        rgb_feature[B_] = rgb_feature[B_] / N;
        //proportion of points having large color components
        rgb_feature_proportional[R_] = rgb_feature_proportional[R_] / N;
        rgb_feature_proportional[G_] = rgb_feature_proportional[G_] / N;
        rgb_feature_proportional[B_] = rgb_feature_proportional[B_] / N;
        //proportion of points having predominant colors
        rgb_feature_predominant[R_] = rgb_feature_predominant[R_] / N;
        rgb_feature_predominant[G_] = rgb_feature_predominant[G_] / N;
        rgb_feature_predominant[B_] = rgb_feature_predominant[B_] / N;
        //HSV
        //mean = sum_x / n
        //stdev = sqrt( sum_x2/n - mean^2 )
        hsv_feature[H_] = hsv_feature[H_] / N;
        hsv_feature[S_] = hsv_feature[S_] / N;
        hsv_feature[V_] = hsv_feature[V_] / N;
        hsv_std[H_] = sqrt( hsv_sum_squared[H_]/N - powf( hsv_feature[H_],2));
        hsv_std[S_] = sqrt( hsv_sum_squared[S_]/N - powf( hsv_feature[S_],2));
        hsv_std[V_] = sqrt( hsv_sum_squared[V_]/N - powf( hsv_feature[V_],2));

        /*
        cout << " cluster: " <<idx<< std::endl;
        cout << "\t H: "<<hsv_feature[H_]<< "\t STD: "<<hsv_std[H_]<<std::endl;
        cout << "\t S: "<<hsv_feature[S_]<< "\t STD: "<<hsv_std[S_]<<std::endl;
        cout << "\t V: "<<hsv_feature[V_]<< "\t STD: "<<hsv_std[V_]<<std::endl;
        waitforGeneric(3);

        cout << "\t"<<rgb_feature[0]<< "\t"<<rgb_feature[0]<< "\t"<<rgb_feature[2]<<std::endl;
        cout << "\t"<<rgb_feature_proportional[0]<< "\t"<<rgb_feature_proportional[1]<< "\t"<<rgb_feature_proportional[2]<<std::endl;
        cout << "\t"<<rgb_feature_predominant[0]<< "\t"<<rgb_feature_predominant[1]<< "\t"<<rgb_feature_predominant[2]<<std::endl;
        */

        //T-shirt (test)
        /*
        if (targetObj_== "cherokee_easy_tee_shirt" && rgb_feature[R_]>th_rgb[R_] && rgb_feature[G_]>th_rgb[G_] && rgb_feature[B_]>th_rgb[B_] && rgb_feature_proportional[G_] >0.3 && rgb_feature_proportional[B_] >0.4){
            object_found_ = true;
            break;
        }*/
        if (targetObj_== "cherokee_easy_tee_shirt" &&  hsv_feature[V_] > 0.2 && hsv_feature[S_] < 0.2 && hsv_std[H_] < 75){
            object_found_ = true;
            break;
        }

        else if ((targetObj_== "kleenex_paper_towels" || targetObj_== "hanes_tube_socks" || targetObj_== "peva_shower_curtain_liner" )
             && hsv_feature[V_] > 0.2 && hsv_feature[S_] < 0.2 && hsv_std[H_] < 120){
            object_found_ = true;
            break;
        }
        //Gloves (test)
        /*
         * if (targetObj_== "womens_knit_gloves" && rgb_feature[R_]< 1/1.6 * th_rgb[R_] && rgb_feature[G_]< 1/1.6 *th_rgb[G_] && rgb_feature[B_]< 1/1.6 *th_rgb[B_] && rgb_feature_proportional[G_] <0.2 && rgb_feature_proportional[B_] <0.2){
            object_found_ = true;
            break;
        }
        */
        if (targetObj_== "womens_knit_gloves" && hsv_feature[V_] < 0.2 && hsv_feature[S_] > 0.01 && hsv_std[H_] < 75){
            object_found_ = true;
            break;
        }
        //Kleenex (test)
        else if (targetObj_== "kleenex_tissue_box" && hsv_feature[H_] > 190 && hsv_feature[H_] < 235 && hsv_feature[V_] > 0.2 && hsv_feature[S_] > 0.2 && hsv_std[H_] < 60){
            object_found_ = true;
            break;
        }
        //scotch_bubble_mailer(test)
        else if (targetObj_== "scotch_bubble_mailer" && hsv_feature[H_] > 30 && hsv_feature[H_] < 80 && hsv_feature[V_] > 0.2 && hsv_feature[S_] > 0.2 && hsv_std[H_] < 105 && N >= 700){
            object_found_ = true;
            break;
        }
        //Plush Bear (test)
        else if (targetObj_== "cloud_b_plush_bear" && hsv_feature[H_] > 30 && hsv_feature[H_] < 80 && hsv_feature[V_] > 0.2 && hsv_feature[S_] > 0.2 && hsv_std[H_] < 105 && N < 700){
            object_found_ = true;
            break;
        }
        //staples_index_cards (test)
        else if (targetObj_== "staples_index_cards" && hsv_feature[H_] > 260 && hsv_feature[H_] < 320 && hsv_feature[V_] > 0.2 && hsv_feature[S_] > 0.2 && hsv_std[H_] < 70 ){
            object_found_ = true;
            break;
        }


        if (publish_output_pc_){
            //Send point clouds to the viewer
            pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2( *claster_cloud_rgb, *cloud_filtered_ptr);
            cloud_filtered_ptr->header = cloud->header;

            pcl_conversions::moveFromPCL(*cloud_filtered_ptr, output);
            pub_output.publish (output);
        }
    }


    /* TBR
      transformed_cloud_rgb = reg.getColoredCloud ();

      if (publish_output_pc_){
          //Send point clouds to the viewer
          pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2);
          pcl::toPCLPointCloud2( *transformed_cloud_rgb, *cloud_filtered_ptr);
          cloud_filtered_ptr->header = cloud->header;

          pcl_conversions::moveFromPCL(*cloud_filtered_ptr, output);
          pub_output.publish (output);
      }*/
    /**/
    // Target object was detected




    if(object_found_) {
        std::cerr<<std::endl;
        //publish the segment found
        if (publish_output_pc_){
            //Send point clouds to the viewer
            pcl::PCLPointCloud2::Ptr cloud_filtered_ptr (new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2( *claster_cloud_rgb, *cloud_filtered_ptr);
            cloud_filtered_ptr->header = cloud->header;

            pcl_conversions::moveFromPCL(*cloud_filtered_ptr, output);
            pub_output.publish (output);
        }

        //Set the tf
        msg_pose.pose.position.x = centroid[0] + camera_to_bin.getOrigin().getX();
        msg_pose.pose.position.y = centroid[1] + camera_to_bin.getOrigin().getY();
        msg_pose.pose.position.z = centroid[2] + camera_to_bin.getOrigin().getZ();
        msg_pose.pose.orientation.x = camera_to_bin.getRotation().getX();
        msg_pose.pose.orientation.y = camera_to_bin.getRotation().getY();
        msg_pose.pose.orientation.z = camera_to_bin.getRotation().getZ();
        msg_pose.pose.orientation.w = camera_to_bin.getRotation().getW();

        //defining the bin side
        if( binID_.at(binID_.size()-1) == 'e'){
           if (  centroid[0] >= 0 )
                result_.side = "right";
            else
                result_.side = "left";
        }
        else{
            if (  centroid[1] <= 0 )
                result_.side = "right";
            else
                result_.side = "left";
            

        }
        // Start the transmission of the tf
        starttransm_ = true;
        objectpose_ = msg_pose;


        ROS_INFO("RGBD: Succeeded, object(%s) found", targetObj_.c_str());

        // unregister any subscribed topic
        sub_.shutdown();

        // set the action state to succeeded
        as_.setSucceeded(result_);

    }
    else if (watchdog_ <= 0) {
        std::cerr<<std::endl;

        ROS_INFO("RGBD: Failed, object(%s) NOT found (timeout)", targetObj_.c_str());


        // unregister any subscribed topic
        sub_.shutdown();

        // set the action state to aborted
        as_.setAborted(result_);

    }
}




/*
   *  this method broadcasts the last detected pose of the target object as tf
  */
void LookForObjectRGBD::threaded_broadcaster_object()
{
    // broadcast the tf if the object has been found
    if (starttransm_ == true)
    {
        std::string objectname_ = targetObj_ + "_final";
        std::string parentframe_ = objectpose_.header.frame_id;
        transform.setOrigin(
                    tf::Vector3(objectpose_.pose.position.x, objectpose_.pose.position.y, objectpose_.pose.position.z));
        transform.setRotation(tf::Quaternion(objectpose_.pose.orientation.x, objectpose_.pose.orientation.y,
                                             objectpose_.pose.orientation.z, objectpose_.pose.orientation.w));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parentframe_, objectname_));
    }
}




/*
   *  stop the code until the system is available :debug
   */
bool LookForObjectRGBD::waitforGeneric (int TIMER){
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
   * filters the input pc around the target bin
   */
sensor_msgs::PointCloud2 LookForObjectRGBD::set_bin_cloud(sensor_msgs::PointCloud2ConstPtr cloud_in){
    sensor_msgs::PointCloud2 cloud_bin;
    Eigen::Matrix4f eigen_transform_toshelf;
    pcl_ros::transformAsMatrix (camera_to_bin, eigen_transform_toshelf);
    pcl_ros::transformPointCloud (eigen_transform_toshelf, *cloud_in, cloud_bin);

    cloud_bin.header.frame_id = binID_;


    // Defining the cropping regions

    float x_cut = 0.12, x_s = -0.02, x_e = 0.42 - x_cut ; //depth
    float y_s, y_e; //width;
    float z_s = 0.015, z_e, z_cut = 0.08; //height
    if(binID_ == "bin_A")	  {
        y_s = -0.11; y_e = -y_s; z_e = 0.23 -z_cut;
    }
    else if(binID_ == "bin_B")  {
        y_s = -0.14; y_e = -y_s; z_e = 0.23 -z_cut;
    }
    else if(binID_ == "bin_C")  {
        y_s = -0.13; y_e = -y_s; z_e = 0.23 -z_cut;
    }
    else if(binID_ == "bin_D")  {
        y_s = -0.11; y_e = -y_s; z_e = 0.18 -z_cut;
    }
    else if(binID_ == "bin_E")  {
        y_s = -0.14; y_e = -y_s; z_e = 0.18 -z_cut;
    }
    else if(binID_ == "bin_F")  {
        y_s = -0.13; y_e = -y_s; z_e = 0.18 -z_cut;
    }
    else if(binID_ == "bin_G")  {
        y_s = -0.11; y_e = -y_s; z_e = 0.18 -z_cut;
    }
    else if(binID_ == "bin_H")  {
        y_s = -0.14; y_e = -y_s; z_e = 0.18 -z_cut;
    }
    else if(binID_ == "bin_I")  {
        y_s = -0.13; y_e = -y_s; z_e = 0.18 -z_cut;
    }
    else if(binID_ == "bin_J")  {
        y_s = -0.11; y_e = -y_s; z_e = 0.22 -z_cut;
    }
    else if(binID_ == "bin_K")  {
        y_s = -0.14; y_e = -y_s; z_e = 0.22 -z_cut;
    }
    else if(binID_ == "bin_L")  {
        y_s = -0.13; y_e = -y_s; z_e = 0.22 -z_cut;
    }
    else if(binID_ == "tote") {
        //x_s = -0.26;   x_e = 0.26;     y_s = -0.28;
        //y_e = 0.08;    z_s = 0.38; 	z_e = 0.67;
        x_s = -0.26;   x_e = 0.26;     y_s = -0.17;
        y_e = 0.17;    z_s = -0.04; 	z_e = 0.3;
    }

    x_roi_min = x_s;
    x_roi_max = x_e;
    y_roi_min = y_s;
    y_roi_max = y_e;
    z_roi_min = z_s;
    z_roi_max = z_e;

    // transform back
    //tfListener->waitForTransform(cloud_in->header.frame_id, binID_, ros::Time::now(), ros::Duration(0.0));
    //tfListener->lookupTransform(cloud_in->header.frame_id, binID_, ros::Time(0), camera_to_bin);

    return cloud_bin;
    /*
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>(pcl_cloud_bin));
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clout_fingered (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB> output;

      pcl::PassThrough<pcl::PointXYZRGB> pass;

      pass.setInputCloud (input_cloud);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits ( x_s, x_e);
      pass.filter (*clout_fingered);
      pass.setInputCloud (clout_fingered);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits ( y_s, y_e);
      pass.filter (*clout_fingered);
      pass.setInputCloud (clout_fingered);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits ( z_s, z_e);
      pass.filter (output);

      output.header.frame_id = binID_;


       transform back
      sensor_msgs::PointCloud2 output_ros_bin;
      pcl::toROSMsg(output, output_ros_bin);

      /// transform back
      sensor_msgs::PointCloud2 output_ros_kinect;
      tf::StampedTransform bin_to_kinect;
      tfListener->waitForTransform(cloud_in->header.frame_id, bin_id, ros::Time::now(), ros::Duration(0.0));
      tfListener->lookupTransform(cloud_in->header.frame_id, bin_id, ros::Time(0), bin_to_kinect);
      Eigen::Matrix4f eigen_transform_tokinect;
      pcl_ros::transformAsMatrix (bin_to_kinect, eigen_transform_tokinect);
      pcl_ros::transformPointCloud (eigen_transform_tokinect, output_ros_bin, output_ros_kinect);
      output_ros_kinect.header.frame_id = cloud_in->header.frame_id;

      return output_ros_kinect;
      */
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "lookforobject_rgbd");

    LookForObjectRGBD lookforobject_rgbd(ros::this_node::getName());

    ROS_INFO("RGBD Action server running");

    ros::Rate r(10);               // 10 hz

    while (ros::ok())
    {
        r.sleep();
        lookforobject_rgbd.threaded_broadcaster_object();
        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}
