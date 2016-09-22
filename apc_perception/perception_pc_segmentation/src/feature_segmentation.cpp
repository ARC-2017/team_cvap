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
#include <feature_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Geometry>
#include <pcl/visualization/pcl_visualizer.h>
#include <windowless_gl_context.h>
#include <hdf5_file.h>
#include <device_1d.h>
#include <utility_kernels_pose.h>
#include <utilities.h>
#include <utility_kernels.h>
#include <multi_rigid_tracker.h>

FeatureSegmentation::FeatureSegmentation(ros::NodeHandle nh)
    : nh_(nh), debug_(false), use_previous_detection_(true), color_only_mode_(false), image_width_(1920), image_height_(1080), min_inliers_(15), min_inliers_redetect_(35)
{
    // get model names from parameter server
    if (!ros::param::get("/feature_segmentation/model_path", model_path_))
        throw std::runtime_error(
                std::string("FeatureSegmentation::FeatureSegmentation: could not "
                            "find /feature_segmentation/model_path on parameter server\n"));

    ROS_INFO_STREAM("FeatureSegmentation model path set to "<<model_path_);

    // find all objects
    bool found_objects = getModelNames(model_path_, model_names_);        
    if (!found_objects){
        throw std::runtime_error(
                    std::string("FeatureSegmentation::FeatureSegmentation: could not "
                                "find any object models in folder") + model_path_);
    }

    // initialize pose publishers
    for (auto it: model_names_){
        pose_publishers_[it] =
            nh_.advertise<geometry_msgs::PoseStamped>("/feature_segmentation/" + it, 1);
    }


    ros::param::get("feature_segmentation/debug_mode", debug_);
    ros::param::get("feature_segmentation/min_inliers", min_inliers_);
    ros::param::get("feature_segmentation/use_previous_detection", use_previous_detection_);
    ros::param::get("feature_segmentation/redetect_inliers", min_inliers_redetect_);

    // initialize model_manager with some default camera parameters
    model_matcher_ = std::unique_ptr<ModelMatcher>(new ModelMatcher(image_width_, image_height_, 250, 250, 500, 500));
    model_matcher_->setMinInliers(min_inliers_);
    model_matcher_->setMinInliersRedetect(min_inliers_redetect_);
    model_matcher_->setDebug(debug_);
    model_matcher_->setUsePreviousDetection(use_previous_detection_);

    float_image_ =
        util::Device2D<float>::Ptr(new util::Device2D<float>(image_width_, image_height_));

    texture_image_ =
        util::Device1D<uchar4>::Ptr(new util::Device1D<uchar4>(image_width_ * image_height_));

    // initialize some random camera intrinsics
    camera_matrix_rgb_ = cv::Mat::zeros(3, 4, CV_64F);

    // setup dynamic reconfigure
    dynamic_reconfigure::Server<perception_pc_segmentation::FeatureConfig>::CallbackType f =
            boost::bind(&FeatureSegmentation::reconfigureCb, this, _1, _2);
    dynamic_reconfigure_server_.setCallback(f);

    ready_ = true;
}

FeatureSegmentation::~FeatureSegmentation() {
}

bool FeatureSegmentation::start() {
    if (!ready_) {
        return false;
    }

    switch_camera_srv_ = nh_.advertiseService(
                "/feature_segmentation/switch_camera", &FeatureSegmentation::switchCameraByName, this);

    switch_objects_srv_ = nh_.advertiseService(
                "/feature_segmentation/switch_objects", &FeatureSegmentation::switchObjects, this);

    stop_tracking_srv_ = nh_.advertiseService(
                "/feature_segmentation/stop_tracking", &FeatureSegmentation::stopTracking, this);


    debug_img_it_.reset(new image_transport::ImageTransport(nh_));
    debug_img_pub_ = debug_img_it_->advertise("/feature_segmentation/image", 1);

    input_rgbs_.clear();
    input_depths_.clear();
    // default camera subscription
    setupCameraSubscribers("kinect_chest");

    return true;
}

bool FeatureSegmentation::switchCameraByName(perception_services::SwitchCameraByNameRequest &req,
                                        perception_services::SwitchCameraByNameResponse &res) {
    ROS_INFO("feature_segmentation switching to camera: %s", req.camera.c_str());
    setupCameraSubscribers(req.camera);
    return true;
}

bool FeatureSegmentation::stopTracking(perception_services::StopTrackingRequest &req,
                                  perception_services::StopTrackingResponse &res) {
    ROS_INFO("feature_segmentation stopping tracking ");
    return unsubscribeFromTopics();
}

bool FeatureSegmentation::unsubscribeFromTopics(){
    // unsubscribe from all camera topics
    sync_rgbd_.reset();
    sync_rgb_.reset();
    sub_depth_.unsubscribe();
    depth_it_.reset();
    sub_rgb_info_.unsubscribe();
    sub_rgb_.unsubscribe();
    rgb_it_.reset();

    return true;
}

bool FeatureSegmentation::switchObjects(perception_services::SwitchObjectsRequest &req,
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
    ROS_INFO_STREAM("feature_segmentation switching to models: ");
    for (auto &it : req.model_names){
        std::cout<<"    "<<it<<std::endl;
    }


    // switch objects
    if (model_ogre_ != nullptr){
        model_ogre_->removeAllModels();
    }
    objects_full_path_.clear();
    objects_names_.clear();
    model_matcher_->removeAllModels();
    for (auto &it : req.model_names){
        objects_full_path_.push_back(composeObjectFilename(it));
//        model_matcher_->addModel(composeObjectInfo(it).c_str());
        model_matcher_->addModelSeparateViews(composeObjectInfo(it).c_str());
        objects_names_.push_back(it);
        if (model_ogre_ != nullptr){
            model_ogre_->addModel(composeObjectFilename(it));
        }
    }

    return true;
}

void FeatureSegmentation::depthAndColorCb(
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

    // update matcher intrinsic matrix
    model_matcher_->updateCalibration(cv_rgb_ptr->image.cols, cv_rgb_ptr->image.rows,
                                     camera_matrix_rgb_.at<double>(0, 2), camera_matrix_rgb_.at<double>(1, 2),
                                     camera_matrix_rgb_.at<double>(0, 0), camera_matrix_rgb_.at<double>(1, 1));

    input_rgbs_.push_back(cv_rgb_ptr->image.clone());
    input_depths_.push_back(cv_depth_ptr->image.clone());

    return;
}

void FeatureSegmentation::colorOnlyCb(
    const sensor_msgs::ImageConstPtr &rgb_msg,
    const sensor_msgs::CameraInfoConstPtr &rgb_info_msg) {
  // we'll assume registration is correct so that rgb and depth camera matrices
  // are equal
  cv::Mat new_camera_matrix_rgb = cv::Mat(3, 4, CV_64F, (void *)rgb_info_msg->P.data()).clone();

  cv_bridge::CvImageConstPtr cv_rgb_ptr, cv_depth_ptr;
  try {
//      cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, "bgr8");
      if (rgb_msg->encoding == "8UC1") // fix for kinect2 mono output
        cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg);
      else
        cv_rgb_ptr = cv_bridge::toCvShare(rgb_msg, "mono8");
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (model_ogre_ == nullptr){
      model_ogre_ = std::unique_ptr<pose::MultipleRigidModelsOgre>( new pose::MultipleRigidModelsOgre(cv_rgb_ptr->image.cols, cv_rgb_ptr->image.rows,
                                                                                                      camera_matrix_rgb_.at<double>(0, 0), camera_matrix_rgb_.at<double>(1, 1),
                                                                                                      camera_matrix_rgb_.at<double>(0, 2), camera_matrix_rgb_.at<double>(1, 2),
                                                                                                      0.01, 10.0));
      for (auto it : objects_full_path_){
          model_ogre_->addModel(it);
      }
  }

  if ((new_camera_matrix_rgb.at<double>(0, 2) != camera_matrix_rgb_.at<double>(0, 2)) ||
      (new_camera_matrix_rgb.at<double>(0, 1) != camera_matrix_rgb_.at<double>(0, 1)) ||
      (new_camera_matrix_rgb.at<double>(0, 0) != camera_matrix_rgb_.at<double>(0, 0)) ||
      (new_camera_matrix_rgb.at<double>(1, 1) != camera_matrix_rgb_.at<double>(1, 1))){
      ROS_WARN_STREAM("Camera intrinsics change -> updating parameters");
      camera_matrix_rgb_ = new_camera_matrix_rgb.clone();
      model_matcher_->updateCalibration(cv_rgb_ptr->image.cols, cv_rgb_ptr->image.rows,
                                       camera_matrix_rgb_.at<double>(0, 2), camera_matrix_rgb_.at<double>(1, 2),
                                       camera_matrix_rgb_.at<double>(0, 0), camera_matrix_rgb_.at<double>(1, 1));
      model_ogre_->updateProjectionMatrix(camera_matrix_rgb_.at<double>(0, 0), camera_matrix_rgb_.at<double>(1, 1),
                                         camera_matrix_rgb_.at<double>(0, 2), camera_matrix_rgb_.at<double>(1, 2),
                                         0.01, 10.0);
  }

  if ((cv_rgb_ptr->image.cols != image_width_) || (cv_rgb_ptr->image.rows != image_height_)){
      // image format changed
      ROS_WARN_STREAM("Input image format changed -> reinitializing render object. Probably leaking memory here.");
      image_width_ = cv_rgb_ptr->image.cols;
      image_height_ = cv_rgb_ptr->image.rows;
      model_matcher_->updateCalibration(cv_rgb_ptr->image.cols, cv_rgb_ptr->image.rows,
                                       camera_matrix_rgb_.at<double>(0, 2), camera_matrix_rgb_.at<double>(1, 2),
                                       camera_matrix_rgb_.at<double>(0, 0), camera_matrix_rgb_.at<double>(1, 1));
  }

  estimatePose(cv_rgb_ptr->image, cv_rgb_ptr, rgb_msg->header.frame_id);

}

void FeatureSegmentation::parameterError(std::string function_name,
                                    std::string topic_name) {
    std::stringstream err;
    err << "FeatureSegmentation::" << function_name << ": could not find "
        << topic_name << " on parameter server" << std::endl;
    throw std::runtime_error(err.str());
}

void FeatureSegmentation::reconfigureCb(perception_pc_segmentation::FeatureConfig &config,
                                        uint32_t level){
    ROS_WARN_STREAM("FeatureSegmentation::Reconfig message received. Setting min_inliers to "<<config.min_inliers);
    ROS_WARN_STREAM("FeatureSegmentation::Setting min_inliers_redetect to "<<config.min_inliers_redetect);
    if (config.debug){
        ROS_WARN_STREAM("FeatureSegmentation::Setting debug to TRUE");
    } else {
        ROS_WARN_STREAM("FeatureSegmentation::Setting debug to FALSE");
    }

    if (config.use_previous_detection){
        ROS_WARN_STREAM("FeatureSegmentation::Setting use_previous_detection_ to TRUE");
    } else {
        ROS_WARN_STREAM("FeatureSegmentation::Setting use_previous_detection_ to FALSE");
    }
    min_inliers_ = config.min_inliers;
    min_inliers_redetect_ = config.min_inliers_redetect;
    debug_ = config.debug;
    use_previous_detection_ = config.use_previous_detection;
    model_matcher_->setMinInliers(min_inliers_);
    model_matcher_->setMinInliersRedetect(min_inliers_redetect_);
    model_matcher_->setDebug(debug_);
    model_matcher_->setUsePreviousDetection(use_previous_detection_);
    return;
}

void FeatureSegmentation::setupCameraSubscribers(std::string camera_name) {

    ROS_INFO_STREAM("FeatureSegmentation subscribing to the camera "<<camera_name);
    // unsubscribe from all camera topics
    sync_rgbd_.reset();
    sub_depth_.unsubscribe();
    depth_it_.reset();
    sub_rgb_info_.unsubscribe();
    sub_rgb_.unsubscribe();
    rgb_it_.reset();

    bool compressed_streams = false;
    ros::param::get("feature_segmentation/use_compressed_streams", compressed_streams);
    if (!compressed_streams){
        ROS_INFO_STREAM("FeatureSegmentation using raw image streams");
    } else {
        ROS_INFO_STREAM("FeatureSegmentation using compressed image streams");
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
    topic_name << "feature_segmentation/camera/" << camera_name << "/rgb";
    std::string rgb_topic;
    if (!ros::param::get(topic_name.str(), rgb_topic)){
        unsubscribeFromTopics();
        parameterError(__func__, topic_name.str());
    }
    topic_name.str("");
    topic_name << "feature_segmentation/camera/" << camera_name << "/rgb_info";
    std::string rgb_info_topic;
    if (!ros::param::get(topic_name.str(), rgb_info_topic)){
        unsubscribeFromTopics();
        parameterError(__func__, topic_name.str());
    }

    rgb_it_.reset(new image_transport::ImageTransport(nh_));
    sub_rgb_.subscribe(*rgb_it_, rgb_topic, 1, rgb_hint);
    sub_rgb_info_.subscribe(nh_, rgb_info_topic, 1);

    topic_name.str("");
    topic_name << "feature_segmentation/camera/" << camera_name << "/color_only_mode";
    if (!ros::param::get(topic_name.str(), color_only_mode_)){
      unsubscribeFromTopics();
      parameterError(__func__, topic_name.str());
    }

    if (color_only_mode_) {
      sync_rgb_.reset(
          new SynchronizerRGB(SyncPolicyRGB(5), sub_rgb_, sub_rgb_info_));
      sync_rgb_->registerCallback(
          boost::bind(&FeatureSegmentation::colorOnlyCb, this, _1, _2));
    } else {
        topic_name.str("");
        topic_name << "feature_segmentation/camera/" << camera_name << "/depth";
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
                    boost::bind(&FeatureSegmentation::depthAndColorCb, this, _1, _2, _3));
    }
}

bool FeatureSegmentation::getModelNames(const std::string& model_path, std::vector<std::string>& model_names){
    using namespace std;
    using namespace boost::filesystem;

    path p(model_path.c_str());

    try
    {
        if (!exists(p)){
            ROS_ERROR_STREAM("FeatureSegmentation::FeatureSegmentation: the model path "<<model_path<<" does not exist");
            throw std::runtime_error(
                        std::string("FeatureSegmentation::FeatureSegmentation: the model_path "
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
        ROS_ERROR_STREAM("FeatureSegmentation::FeatureSegmentation: error while looking for object models "<<ex.what());
        throw std::runtime_error(
                    std::string("error while looking for object models ") + ex.what());
    }

    if (!model_names.size()){
        return false;
    } else {
        ROS_INFO_STREAM("FeatureSegmentation found the following objects:");
        for (auto model_name : model_names){
            cout << "    " << model_name << '\n';
        }
    }

    return true;

}

std::string FeatureSegmentation::composeObjectFilename(std::string model_name){
    using namespace std;
    using namespace boost::filesystem;

    string full_name = (model_path_ + "/" + model_name + "/" + model_name + ".obj");

    path p(full_name.c_str());

    try
    {
        if (!exists(p)){
            ROS_ERROR_STREAM("FeatureSegmentation::FeatureSegmentation: the model name "<<full_name<<" does not exist");
            throw std::runtime_error(
                        std::string("FeatureSegmentation::FeatureSegmentation: the model name does not exist ") + full_name);
        }
    } catch (const filesystem_error& ex)
    {
        ROS_ERROR_STREAM("FeatureSegmentation::FeatureSegmentation: error while looking for object model "<<ex.what());
        throw std::runtime_error(
                    std::string("error while looking for object model ") + ex.what());
    }

    return full_name;
}

std::string FeatureSegmentation::composeObjectInfo(std::string model_name){
    using namespace std;
    using namespace boost::filesystem;

    string full_name = (model_path_ + "/" + model_name + "/" + model_name + "_SIFT.h5");

    path p(full_name.c_str());

    try
    {
        if (!exists(p)){
            ROS_ERROR_STREAM("FeatureSegmentation::FeatureSegmentation: the model name "<<full_name<<" does not exist");
            throw std::runtime_error(
                        std::string("FeatureSegmentation::FeatureSegmentation: the model name does not exist ") + full_name);
        }
    } catch (const filesystem_error& ex)
    {
        ROS_ERROR_STREAM("FeatureSegmentation::FeatureSegmentation: error while looking for object model "<<ex.what());
        throw std::runtime_error(
                    std::string("error while looking for object model ") + ex.what());
    }

    return full_name;
}



void FeatureSegmentation::estimatePose(cv::Mat rgb_image, cv_bridge::CvImageConstPtr cv_rgb_ptr, const std::string& frame_id){
    if (debug_){
        ROS_INFO_STREAM("Looking for "<<model_matcher_->getNumberOfObjectsSeparateViews()<<" objects ");
    }
    if (!model_matcher_->getNumberOfObjectsSeparateViews() && !model_matcher_->getNumberOfObjects()){
        if (debug_){
//            ROS_ERROR_STREAM("FeatureSegmentation -- No objects added, not estimating any poses.");
        }
        return;
    }

    std::map<int,pose::TranslationRotation3D> poses;
    if (model_matcher_->getNumberOfObjectsSeparateViews()){
        poses = model_matcher_->estimatePoseAllObjectsSeparateViews(rgb_image);
    } else {
        poses = model_matcher_->estimatePoseAllObjects(rgb_image);
    }

    if (!poses.size()){
        return;
    }

    if (debug_){
        ROS_INFO_STREAM("Matching finished. Poses size "<<poses.size());
    }

    cv::Mat image_float;
    rgb_image.convertTo(image_float, CV_32FC1);

    // copy new image to device
    cudaMemcpy2D(float_image_->data(), float_image_->pitch(),
                 image_float.data, image_float.cols * sizeof(float),
                 image_float.cols * sizeof(float), image_float.rows,
                 cudaMemcpyHostToDevice);

    double near_plane = 0.01; // for init only
    double far_plane = 10.0;  // for init only
    double width = rgb_image.cols;
    double height = rgb_image.rows;

    std::vector<pose::TranslationRotation3D> poses_vector;
    for (auto it: poses){
        // add models and set poses
        poses_vector.push_back(it.second);
    }
        model_ogre_->render(poses_vector);

        vision::blendFloatImageFloatArrayToRGBA(
            texture_image_->data(), float_image_->data(),
            model_ogre_->getTexture(), width * sizeof(uchar4),
            float_image_->pitch(), width, height);

        cv::Mat texture = cv::Mat::zeros(height, width, CV_8UC4);
        cudaMemcpy(texture.data, texture_image_->data(),
                   width * height * sizeof(uchar4),
                   cudaMemcpyDeviceToHost);

        debug_img_pub_.publish(
        cv_bridge::CvImage(std_msgs::Header(), "rgba8", texture).toImageMsg());

        for (auto it: poses){

        // publish pose on TF
        pose::TranslationRotation3D object_pose = it.second;

        if (!object_pose.isValid()){
            continue;
        }
        double t[3];
        object_pose.getT(t);
        double x, y, z, w;
        object_pose.getQuaternion(x, y, z, w);

        geometry_msgs::Pose pose;
        pose.position.x = t[0];
        pose.position.y = t[1];
        pose.position.z = t[2];
        pose.orientation.x = x;
        pose.orientation.y = y;
        pose.orientation.z = z;
        pose.orientation.w = w;

        tf::StampedTransform object_transform;
        object_transform.setOrigin(tf::Vector3(
            pose.position.x, pose.position.y, pose.position.z));
        object_transform.setRotation(
            tf::Quaternion(pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w));
        object_transform.stamp_ = ros::Time::now();
//        object_transform.stamp_ = cv_rgb_ptr->header.stamp;
//        object_transform.frame_id_ = cv_rgb_ptr->header.frame_id;
        object_transform.frame_id_ = frame_id;
        object_transform.child_frame_id_ = objects_names_.at(it.first);
        tfb_.sendTransform(object_transform);

        geometry_msgs::PoseStamped curr_pose_stamped;
        curr_pose_stamped.pose = pose;
        curr_pose_stamped.header.frame_id = frame_id;
        curr_pose_stamped.header.stamp =  ros::Time::now();;
        pose_publishers_[objects_names_.at(it.first)]
            .publish(curr_pose_stamped);

      }
}
