#ifndef __RGBD_SERVER__
#define __RGBD_SERVER__


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
#include <actionlib/server/simple_action_server.h>
//#include <geometry_msgs/WrenchStamped.h> //TBR
//#include <geometry_msgs/TwistStamped.h>  //TBR

//Math
#include <Eigen/Dense>
#include <math.h>

//Actions and threading
#include <apc_objects_detection_action/LookForObjectAction.h>
#include <boost/thread.hpp>


//PCL related //@TOCNG put it on a different class
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include "pcl_ros/impl/transforms.hpp"
#include <pcl/visualization/pcl_visualizer.h>

//PCL visualization
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//color segmentation
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>


//watchdog timer
#define TIME_OUT_SENS 20

//RGB and HSV index
#define R_ 0
#define G_ 1
#define B_ 2
#define H_ 0
#define S_ 1
#define V_ 2

//transmission tf control
#define STOP_TRANSMISSION -1

//Debug
#include <ctime>



class LookForObjectRGBD
{
	public:
		LookForObjectRGBD(std::string name);
		~LookForObjectRGBD();

		void goalCB();
		void preemptCB();
		void analysisCB(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

		std::string setCameraTopic( std::string camera);
		void threaded_broadcaster_object();
		bool waitforGeneric (int TIMER);
		sensor_msgs::PointCloud2 set_bin_cloud(sensor_msgs::PointCloud2ConstPtr cloud_in);

	private:

	protected:
	  ros::NodeHandle nh_;

	  geometry_msgs::PoseStamped objectpose_;
	  actionlib::SimpleActionServer<apc_objects_detection_action::LookForObjectAction> as_;

	  //analysis variables
	  std::string quality_;
	  std::string camera_topic_;
      bool debug_,publish_output_pc_;

	  // perception settings and action_msgs
	  std::string action_name_;
	  std_msgs::String msg_str;
      std::vector<std::string> objectslist_;
      std::string targetObj_, param_;
	  std::string cameraID_, methodID_, taskID_, binID_;
	  int goal_, watchdog_;
	  bool object_found_, starttransm_;

	  apc_objects_detection_action::LookForObjectFeedback feedback_;
	  apc_objects_detection_action::LookForObjectResult result_;
	  apc_objects_detection_action::LookForObjectGoal new_goal_;
	  ros::Subscriber sub_;

	  // TF transmission related
	  tf::TransformBroadcaster br;
	  tf::Transform transform;
	  tf::TransformListener* tfListener;
	  tf::StampedTransform camera_to_bin;

	  //Point clouds temporary variable for ROS-publish

	  sensor_msgs::PointCloud2 output;
	  ros::Publisher pub_output;

	  //pcl analysis
	  float x_roi_min ;           //Define the ROI xmin
	  float x_roi_max ;            //Define the ROI xmax
	  float z_roi_min ;            //Define the ROI zmin
	  float z_roi_max ;            //Define the ROI zmax
	  float y_roi_min ;            //Define the ROI ymin
	  float y_roi_max ;            //Define the ROI ymax
};


#endif // __RGBD_SERVER__
