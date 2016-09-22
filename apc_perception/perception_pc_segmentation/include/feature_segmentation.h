#ifndef __FEATURE_SEGMENTATION__H
#define __FEATURE_SEGMENTATION__H

#include <thread>
#include <atomic>
#include <mutex>
#include <boost/filesystem.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <perception_services/SwitchObjects.h>
#include <perception_services/SwitchCameraByName.h>
#include <perception_services/StopTracking.h>
#include <perception_pc_segmentation/FeatureConfig.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "model_matcher.h"

#include <multiple_rigid_models_ogre.h>
#include <device_1d.h>
#include <device_2d.h>

class FeatureSegmentation {
public:
    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;


    FeatureSegmentation(ros::NodeHandle nh);

    ~FeatureSegmentation();

    bool start();

private:
    // methods
    bool switchCameraByName(perception_services::SwitchCameraByNameRequest &req,
                            perception_services::SwitchCameraByNameResponse &res);

    bool switchObjects(perception_services::SwitchObjectsRequest &req,
                       perception_services::SwitchObjectsResponse &res);

    bool stopTracking(perception_services::StopTrackingRequest &req,
                      perception_services::StopTrackingResponse &res);

    bool unsubscribeFromTopics();

    void depthAndColorCb(const sensor_msgs::ImageConstPtr &depth_msg,
                         const sensor_msgs::ImageConstPtr &rgb_msg,
                         const sensor_msgs::CameraInfoConstPtr &rgb_info_msg);


    void colorOnlyCb(const sensor_msgs::ImageConstPtr &rgb_msg,
                 const sensor_msgs::CameraInfoConstPtr &rgb_info_msg);

    void reconfigureCb(perception_pc_segmentation::FeatureConfig &config,
                       uint32_t level);

    void setupCameraSubscribers(std::string camera_name);

    void parameterError(std::string function_name, std::string topic_name);

    bool getModelNames(const std::string& model_path, std::vector<std::string>& model_names);
    std::string composeObjectFilename(std::string model_name);
    std::string composeObjectInfo(std::string model_name);


    CloudPtr filterInputImages(const std::vector<cv::Mat>& input_rgbs,
                               const std::vector<cv::Mat>& inpud_depths,
                               const cv::Mat& intrinsics,
                               const std::string& filter_type);

    std::vector<CloudPtr> getConvexSegments(CloudPtr input,
                                             pcl::PointCloud<pcl::Normal>::Ptr);

    void estimatePose(cv::Mat rgb_image, cv_bridge::CvImageConstPtr cv_rgb_ptr, const std::string& frame_id);

    // members
    ros::NodeHandle nh_;
    bool ready_;
    bool debug_;
    bool use_previous_detection_;
    std::string model_path_;
    std::vector<std::string> obj_filenames_;
    std::vector<std::string> model_names_;
    std::vector<std::string> objects_full_path_;
    std::vector<std::string> objects_names_;
    std::unique_ptr<ModelMatcher> model_matcher_;

    // input filtering
    std::string input_filter_type_;
    int input_filter_size_;
    double max_depth_cutoff_;
    // input data
    std::vector<cv::Mat> input_rgbs_;
    std::vector<cv::Mat> input_depths_;

    // intrinsics
    cv::Mat camera_matrix_rgb_;
    int image_width_, image_height_;
    int min_inliers_;
    int min_inliers_redetect_;


    // Services
    ros::ServiceServer switch_objects_srv_;
    ros::ServiceServer switch_camera_srv_;
    ros::ServiceServer stop_tracking_srv_;

    // Publishers
    boost::shared_ptr<image_transport::ImageTransport> debug_img_it_;
    image_transport::Publisher debug_img_pub_;
    tf::TransformBroadcaster tfb_;
    std::map<std::string, ros::Publisher> pose_publishers_;

    // Subscriptions
    ros::Subscriber sub_detector_pose_;
    boost::shared_ptr<image_transport::ImageTransport> rgb_it_, depth_it_;
    image_transport::SubscriberFilter sub_depth_, sub_rgb_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_rgb_info_;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>
    SyncPolicyRGBD;
    typedef message_filters::Synchronizer<SyncPolicyRGBD> SynchronizerRGBD;
    boost::shared_ptr<SynchronizerRGBD> sync_rgbd_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::CameraInfo> SyncPolicyRGB;
    typedef message_filters::Synchronizer<SyncPolicyRGB> SynchronizerRGB;
    boost::shared_ptr<SynchronizerRGB> sync_rgb_;
    bool color_only_mode_;

    // dynamic reconfigure
    dynamic_reconfigure::Server<perception_pc_segmentation::FeatureConfig> dynamic_reconfigure_server_;


    // render object
//    pose::MultipleRigidModelsOgre model_ogre_;
    std::unique_ptr<pose::MultipleRigidModelsOgre> model_ogre_;

    // low-level image types
    util::Device2D<float>::Ptr float_image_;
    util::Device1D<uchar4>::Ptr texture_image_;
};

#endif
