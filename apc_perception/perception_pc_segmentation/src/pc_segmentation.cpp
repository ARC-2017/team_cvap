#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pc_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
//#include <object_3d_retrieval/supervoxel_segmentation.h>
//#include <convex_segmentation/CloudArray.h>

pcl::visualization::PCLVisualizer *viz_g;

PCSegmentation::PCSegmentation(ros::NodeHandle nh)
    : nh_(nh), debug_(false){
    // get model names from parameter server
    if (!ros::param::get("/pc_segmentation/model_path", model_path_))
        throw std::runtime_error(
                std::string("PCSegmentation::PCSegmentation: could not "
                            "find /pc_segmentation/model_path on parameter server\n"));

    ROS_INFO_STREAM("PCSegmentation model path set to "<<model_path_);

    // find all objects
    bool found_objects = getModelNames(model_path_, model_names_);
    if (!found_objects){
        throw std::runtime_error(
                    std::string("PCSegmentation::PCSegmentation: could not "
                                "find any object models in folder") + model_path_);
    }

    ros::param::get("pc_segmentation/input_filter_type", input_filter_type_);
    if ((input_filter_type_ != "mean") && (input_filter_type_!= "median")){
        throw std::runtime_error(
                    std::string("PCSegmentation::PCSegmentation: unknwon "
                                "input filter type ") + input_filter_type_ + std::string(" -- supported types [mean, median]"));
    }

    ros::param::get("pc_segmentation/input_filter_size", input_filter_size_);
    ros::param::get("pc_segmentation/max_depth", max_depth_cutoff_);
    ros::param::get("pc_segmentation/debug_mode", debug_);

    if (debug_){
        viz_g = new pcl::visualization::PCLVisualizer ("PC Segmentation");
    }

    ready_ = true;
}

PCSegmentation::~PCSegmentation() {
}

bool PCSegmentation::start() {
    if (!ready_) {
        return false;
    }

    switch_camera_srv_ = nh_.advertiseService(
                "/pc_segmentation/switch_camera", &PCSegmentation::switchCameraByName, this);

    switch_objects_srv_ = nh_.advertiseService(
                "/pc_segmentation/switch_objects", &PCSegmentation::switchObjects, this);

    stop_tracking_srv_ = nh_.advertiseService(
                "/pc_segmentation/stop_tracking", &PCSegmentation::stopTracking, this);


    debug_img_it_.reset(new image_transport::ImageTransport(nh_));
    debug_img_pub_ = debug_img_it_->advertise("/pc_segmentation/image", 1);

    input_rgbs_.clear();
    input_depths_.clear();
    // default camera subscription
    setupCameraSubscribers("kinect_chest");

    return true;
}

bool PCSegmentation::switchCameraByName(perception_services::SwitchCameraByNameRequest &req,
                                        perception_services::SwitchCameraByNameResponse &res) {
    ROS_INFO("pc_segmentation switching to camera: %s", req.camera.c_str());
    setupCameraSubscribers(req.camera);
    return true;
}

bool PCSegmentation::stopTracking(perception_services::StopTrackingRequest &req,
                                  perception_services::StopTrackingResponse &res) {
    ROS_INFO("pc_segmentation stopping tracking ");
    return unsubscribeFromTopics();
}

bool PCSegmentation::unsubscribeFromTopics(){
    // unsubscribe from all camera topics
    sync_rgbd_.reset();
    sub_depth_.unsubscribe();
    depth_it_.reset();
    sub_rgb_info_.unsubscribe();
    sub_rgb_.unsubscribe();
    rgb_it_.reset();

    return true;
}

bool PCSegmentation::switchObjects(perception_services::SwitchObjectsRequest &req,
                                   perception_services::SwitchObjectsResponse &res) {
    // error checking -> lookup each object name in original object set

    for (auto requested_object : req.model_names) {
        bool found = false;
        for (auto original_object : model_names_){
            if (original_object==requested_object){
                found = true;
                break;
            }
        }

        if (!found){
            ROS_ERROR("ERROR switching objects, could not find %s", requested_object.c_str());
            throw std::runtime_error(
                        std::string("ERROR switching objects, could not find ") + requested_object);
        }
    }

    // object switching
    std::stringstream ss;
    ROS_INFO_STREAM("pc_segmentation switching to models: ");
    for (auto &it : req.model_names){
        std::cout<<"    "<<it<<std::endl;
    }


    // switch objects
    objects_full_path_.clear();
    for (auto &it : req.model_names){
        objects_full_path_.push_back(composeObjectFilename(it));
    }

    return true;
}

void PCSegmentation::depthAndColorCb(
        const sensor_msgs::ImageConstPtr &depth_msg,
        const sensor_msgs::ImageConstPtr &rgb_msg,
        const sensor_msgs::CameraInfoConstPtr &rgb_info_msg) {
    // we'll assume registration is correct so that rgb and depth camera matrices
    // are equal
    camera_matrix_rgb_ = cv::Mat(3, 4, CV_64F, (void *)rgb_info_msg->P.data()).clone();

    cv_bridge::CvImageConstPtr cv_rgb_ptr, cv_depth_ptr;
    try {
        cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
        cv_depth_ptr = cv_bridge::toCvShare(depth_msg,"16UC1");
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    input_rgbs_.push_back(cv_rgb_ptr->image.clone());
    input_depths_.push_back(cv_depth_ptr->image.clone());

    if ((input_rgbs_.size() == input_filter_size_) ||
            (input_filter_size_ == -1)) {
        ROS_INFO_STREAM("PCSegmentation collected enough images. Proceeding");
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> input_cloud;
        input_cloud =filterInputImages(input_rgbs_,
                                       input_depths_,
                                       camera_matrix_rgb_,
                                       input_filter_type_);
        input_rgbs_.clear();
        input_depths_.clear();

        if (debug_){
            viz_g->addCoordinateSystem();
            viz_g->addPointCloud(input_cloud, "input_cloud_temp");
            viz_g->spin();
            viz_g->removeAllPointClouds();
        }

        ROS_INFO_STREAM("PCSegmentation finished filtering input point cloud.");
        // computing normals
        pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud(input_cloud);
        normal_estimation.setRadiusSearch(0.02);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(input_cloud);
        normal_estimation.setSearchMethod(tree);
        normal_estimation.compute(*normals);
        ROS_INFO_STREAM("PCSegmentation ---- starting convex segmentation.");

        auto convex_segments = getConvexSegments(input_cloud, normals);

//        if (debug_){
//            for (auto segment : convex_segments){
//                viz_g->addCoordinateSystem();
//                viz_g->addPointCloud(segment, "input_segment");
//                viz_g->spin();
//                viz_g->removeAllPointClouds();
//            }
//        }

    } else {
        return;
    }


}

void PCSegmentation::parameterError(std::string function_name,
                                    std::string topic_name) {
    std::stringstream err;
    err << "PCSegmentation::" << function_name << ": could not find "
        << topic_name << " on parameter server" << std::endl;
    throw std::runtime_error(err.str());
}


void PCSegmentation::setupCameraSubscribers(std::string camera_name) {

    ROS_INFO_STREAM("PCSegmentation subscribing to the camera "<<camera_name);
    // unsubscribe from all camera topics
    sync_rgbd_.reset();
    sub_depth_.unsubscribe();
    depth_it_.reset();
    sub_rgb_info_.unsubscribe();
    sub_rgb_.unsubscribe();
    rgb_it_.reset();

    bool compressed_streams = false;
    ros::param::get("pc_segmentation/use_compressed_streams", compressed_streams);
    if (!compressed_streams){
        ROS_INFO_STREAM("PCSegmentation using raw image streams");
    } else {
        ROS_INFO_STREAM("PCSegmentation using compressed image streams");
    }

    image_transport::TransportHints rgb_hint, depth_hint;
    if (compressed_streams) {
        rgb_hint = image_transport::TransportHints("compressed");
        depth_hint = image_transport::TransportHints("compressedDepth");
    } else {
        rgb_hint = image_transport::TransportHints("raw");
        depth_hint = image_transport::TransportHints("raw");
    }

    // fetch rgb topic names from parameter server
    std::stringstream topic_name;
    topic_name << "pc_segmentation/camera/" << camera_name << "/rgb";
    std::string rgb_topic;
    if (!ros::param::get(topic_name.str(), rgb_topic)){
        unsubscribeFromTopics();
        parameterError(__func__, topic_name.str());
    }
    topic_name.str("");
    topic_name << "pc_segmentation/camera/" << camera_name << "/rgb_info";
    std::string rgb_info_topic;
    if (!ros::param::get(topic_name.str(), rgb_info_topic)){
        unsubscribeFromTopics();
        parameterError(__func__, topic_name.str());
    }

    rgb_it_.reset(new image_transport::ImageTransport(nh_));
    sub_rgb_.subscribe(*rgb_it_, rgb_topic, 1, rgb_hint);
    sub_rgb_info_.subscribe(nh_, rgb_info_topic, 1);

    // NOTE: the camera frame is not used here
    //  topic_name.str("");
    //  topic_name << "/camera/" << camera_index << "/robot_frame";
    //  if (!ros::param::get(topic_name.str(), robot_camera_frame_id_))
    //    parameterError(__func__, topic_name.str());

    topic_name.str("");

    topic_name.str("");
    topic_name << "pc_segmentation/camera/" << camera_name << "/depth";
    std::string depth_topic;
    if (!ros::param::get(topic_name.str(), depth_topic)){
        unsubscribeFromTopics();
        parameterError(__func__, topic_name.str());
    }

    depth_it_.reset(new image_transport::ImageTransport(nh_));
    sub_depth_.subscribe(*depth_it_, depth_topic, 1, depth_hint);
    sync_rgbd_.reset(new SynchronizerRGBD(SyncPolicyRGBD(5), sub_depth_,
                                          sub_rgb_, sub_rgb_info_));
    sync_rgbd_->registerCallback(
                boost::bind(&PCSegmentation::depthAndColorCb, this, _1, _2, _3));
}

bool PCSegmentation::getModelNames(const std::string& model_path, std::vector<std::string>& model_names){
    using namespace std;
    using namespace boost::filesystem;

    path p(model_path.c_str());

    try
    {
        if (!exists(p)){
            ROS_ERROR_STREAM("PCSegmentation::PCSegmentation: the model path "<<model_path<<" does not exist");
            throw std::runtime_error(
                        std::string("PCSegmentation::PCSegmentation: the model_path "
                                    " does not exist \n"));
            return false;
        }

        std::list<directory_entry> object_folders;
        boost::copy( boost::make_iterator_range(directory_iterator(p), {}),
                     std::back_inserter(object_folders) );

        for (auto& x : object_folders){
            model_names.push_back(x.path().filename().string());
        }

    }
    catch (const filesystem_error& ex)
    {
        ROS_ERROR_STREAM("PCSegmentation::PCSegmentation: error while looking for object models "<<ex.what());
        throw std::runtime_error(
                    std::string("error while looking for object models ") + ex.what());
    }

    if (!model_names.size()){
        return false;
    } else {
        ROS_INFO_STREAM("PCSegmentation found the following objects:");
        for (auto model_name : model_names){
            cout << "    " << model_name << '\n';
        }
    }

    return true;

}

std::string PCSegmentation::composeObjectFilename(std::string model_name){
    using namespace std;
    using namespace boost::filesystem;

    string full_name = (model_path_ + "/" + model_name + "/" + model_name + ".obj");

    path p(full_name.c_str());

    try
    {
        if (!exists(p)){
            ROS_ERROR_STREAM("PCSegmentation::PCSegmentation: the model name "<<full_name<<" does not exist");
            throw std::runtime_error(
                        std::string("PCSegmentation::PCSegmentation: the model name does not exist ") + full_name);
        }
    } catch (const filesystem_error& ex)
    {
        ROS_ERROR_STREAM("PCSegmentation::PCSegmentation: error while looking for object model "<<ex.what());
        throw std::runtime_error(
                    std::string("error while looking for object model ") + ex.what());
    }

    return full_name;
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> PCSegmentation::filterInputImages(const std::vector<cv::Mat>& input_rgbs,
                                                                                       const std::vector<cv::Mat>& input_depths,
                                                                                       const cv::Mat& intrinsics,
                                                                                       const std::string& filter_type){
    using namespace std;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    filtered->is_dense = true;

    float fx = (float)intrinsics.at<double>(0, 0);
    float fy = (float)intrinsics.at<double>(1, 1);
    float cx = (float)intrinsics.at<double>(0, 2);
    float cy = (float)intrinsics.at<double>(1, 2);

    int width = input_rgbs[0].cols;
    int height = input_rgbs[0].rows;

    for (size_t i=0; i<height; i++){
        for (size_t j=0; j<width; j++){
            pcl::PointXYZRGB pt;

            // for depth, average according to input_filter_type_ from all depth images
            vector<float> point_depths;
            for (size_t k=0; k<input_depths.size(); k++){
                const u_int16_t point_depth = input_depths[k].at<u_int16_t>(i,j);
                if (point_depth != 0){
                    point_depths.push_back((float)point_depth);
                }
            }
            if (point_depths.size() == 0){ // all invalid points
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                filtered->is_dense = false;
            } else {
                float final_depth = 0.0;
                if (filter_type == "median"){
                    std::sort(point_depths.begin(), point_depths.end());
                    int mid_depth_index = point_depths.size() / 2;
                    final_depth = point_depths[mid_depth_index];
                } else { // filter type "mean"
                    float sum_of_depths = 0.0;
                    for (auto temp : point_depths) { sum_of_depths += temp;}
                    final_depth = sum_of_depths / point_depths.size();
                }

                // convert to world coordinates
                final_depth *= 0.001; // convert to meters from uint16
                if (final_depth > max_depth_cutoff_){
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                } else {
                    pt.x = (j - cx) * final_depth / fx;
                    pt.y = (i - cy) * final_depth / fy;
                    pt.z = final_depth;
                }
            }

            // set color from rgb image (not filtered, just choose middle image)
            int mid_index = input_rgbs.size()/2;
            uint32_t point_rgb = ((uint8_t)input_rgbs[mid_index].at<cv::Vec3b>(i, j)[2] << 16 | (uint8_t) input_rgbs[mid_index].at<cv::Vec3b>(i, j)[1] << 8 | (uint8_t)input_rgbs[mid_index].at<cv::Vec3b>(i, j)[0]);
            pt.rgb = *reinterpret_cast<float*>(&point_rgb);

            filtered->push_back(pt);
        }
    }

    //    // set cloud size (for isOrganized flag)
    filtered->width = width;
    filtered->height = height;


    return filtered;
}

std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> PCSegmentation::getConvexSegments(CloudPtr input, pcl::PointCloud<pcl::Normal>::Ptr normals){
    using namespace std;
    // compute convex segmentation
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> toRet;
    return toRet;
}
